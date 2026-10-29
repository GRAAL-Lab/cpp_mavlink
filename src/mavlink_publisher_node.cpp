// C++ counterpart of py_mavlink/publisher_mavlink_msg.py (as described in the README).
// Listens on UDP (default 0.0.0.0:14551) and publishes decoded telemetry.
//
// NOTE: the original publishes custom types from the `mavlink_msg` package. Their
// .msg definitions were not available when this port was written, so standard
// ROS types are used instead (see the mapping table in the accompanying notes).
#include <algorithm>
#include <atomic>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "cpp_mavlink/mavlink_udp.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "mavros_msgs/msg/rc_in.hpp"
#include "mavros_msgs/msg/rc_out.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"

#include "mavlink_msg/msg/attitude.hpp"
#include "mavlink_msg/msg/imu.hpp"


using namespace std::chrono_literals;

class MavlinkPublisher : public rclcpp::Node
{
public:
  MavlinkPublisher()
  : Node("mavlink_publisher")
  {
    const auto addr = declare_parameter<std::string>("bind_address", "0.0.0.0");
    const auto port = declare_parameter<int>("port", 14551);
    const auto prefix = declare_parameter<std::string>("topic_prefix", "/bluerov2");
    frame_id_ = declare_parameter<std::string>("frame_id", "base_link");

    link_ = std::make_unique<cpp_mavlink::MavlinkUdp>(cpp_mavlink::MavlinkUdp::Mode::kListen, addr, static_cast<uint16_t>(port));

    //attitude_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(prefix + "/attitude", 10);
    //attitude_rate_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(prefix + "/attitude_rate", 10);
    attitudePub_ = create_publisher<mavlink_msg::msg::Attitude>(prefix + "/attitude", 10);

    battery_pub_ = create_publisher<sensor_msgs::msg::BatteryState>(prefix + "/battery_status", 10);
    rc_pub_ = create_publisher<mavros_msgs::msg::RCIn>(prefix + "/rc_channels", 10);
    servo_pub_ = create_publisher<mavros_msgs::msg::RCOut>(prefix + "/servo_output", 10);

    //raw_imu_pub_ = create_publisher<std_msgs::msg::Int16MultiArray>(prefix + "/raw_imu", 10);
    rawImuPub_ = create_publisher<mavlink_msg::msg::IMU>(prefix + "/raw_imu", 10);
    rawImuTempPub_ = create_publisher<sensor_msgs::msg::Temperature>(prefix + "/raw_imu/temperature", 10);

    imu2_pub_ = create_publisher<sensor_msgs::msg::Imu>(prefix + "/scaled_imu2", 10);
    imu2_mag_pub_ = create_publisher<sensor_msgs::msg::MagneticField>(prefix + "/scaled_imu2/mag", 10);
    imu2_temp_pub_ = create_publisher<sensor_msgs::msg::Temperature>(prefix + "/scaled_imu2/temperature", 10);
    press_pub_ = create_publisher<sensor_msgs::msg::FluidPressure>(prefix + "/scaled_pressure", 10);
    press_temp_pub_ = create_publisher<sensor_msgs::msg::Temperature>(prefix + "/scaled_pressure/temperature", 10);
    press2_pub_ = create_publisher<sensor_msgs::msg::FluidPressure>(prefix + "/scaled_pressure2", 10);
    press2_temp_pub_ = create_publisher<sensor_msgs::msg::Temperature>(prefix + "/scaled_pressure2/temperature", 10);

    running_ = true;
    thread_ = std::thread([this]() {loop();});
    RCLCPP_INFO(get_logger(), "Listening for MAVLink on udp://%s:%d", addr.c_str(), port);

    RCLCPP_INFO(get_logger(), "Waiting for a heartbeat...");
    mavlink_message_t msg;
    bool got_heartbeat = false;
    while (running_ && rclcpp::ok()) {
      if (!link_->receive(msg)) {continue;}
      if (!got_heartbeat) {
        if (msg.msgid != MAVLINK_MSG_ID_HEARTBEAT) {continue;}
        got_heartbeat = true;
        RCLCPP_INFO(get_logger(), "Heartbeat received! (system %d, component %d)",
          msg.sysid, msg.compid);
        continue;
      }
      handle(msg);
    }

  }

  ~MavlinkPublisher() override
  {
    running_ = false;
    if (thread_.joinable()) {thread_.join();}
  }

private:
  static constexpr double kG = 9.80665;  // m/s^2 per g

  std_msgs::msg::Header header() const
  {
    std_msgs::msg::Header h;
    h.stamp = now();
    h.frame_id = frame_id_;
    return h;
  }

  void loop()
  {
    mavlink_message_t msg;
    while (running_ && rclcpp::ok()) {
      if (link_->receive(msg)) {handle(msg);}
    }
  }

  void publish_pressure(
    const rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr & pp,
    const rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr & tp,
    float press_abs_hpa, int16_t temp_cdeg)
  {
    sensor_msgs::msg::FluidPressure p;
    p.header = header();
    p.fluid_pressure = press_abs_hpa * 100.0;  // hPa -> Pa
    p.variance = 0.0;                           // unknown
    pp->publish(p);
    sensor_msgs::msg::Temperature t;
    t.header = p.header;
    t.temperature = temp_cdeg * 0.01;           // cdegC -> degC
    t.variance = 0.0;
    tp->publish(t);
  }

  void handle(const mavlink_message_t & m)
  {
    switch (m.msgid) {
      case MAVLINK_MSG_ID_ATTITUDE: {
          mavlink_attitude_t a;
          mavlink_msg_attitude_decode(&m, &a);
          geometry_msgs::msg::Vector3Stamped v;
          mavlink_msg::msg::Attitude att;
//          v.header = header();
//          v.vector.x = a.roll; v.vector.y = a.pitch; v.vector.z = a.yaw;      // rad
//          attitude_pub_->publish(v);
//          v.vector.x = a.rollspeed; v.vector.y = a.pitchspeed; v.vector.z = a.yawspeed;  // rad/s
//          attitude_rate_pub_->publish(v);

          att.header = header();
          att.roll = a.roll;
          att.roll  = a.roll; att.pitch = a.pitch; att.yaw = a.yaw;      // rad
          att.roll_speed = a.rollspeed; att.pitch_speed  = a.pitchspeed; att.yaw_speed  = a.yawspeed;  // rad/s

          attitudePub_->publish(att);
          break;
        }
      case MAVLINK_MSG_ID_BATTERY_STATUS: {
          mavlink_battery_status_t b;
          mavlink_msg_battery_status_decode(&m, &b);
          sensor_msgs::msg::BatteryState s;
          s.header = header();
          const float nan = std::numeric_limits<float>::quiet_NaN();
          double v_sum = 0.0;
          for (int i = 0; i < 10; ++i) {
            if (b.voltages[i] != UINT16_MAX) {
              s.cell_voltage.push_back(b.voltages[i] * 1e-3f);  // mV -> V
              v_sum += b.voltages[i] * 1e-3;
            }
          }
          s.voltage = s.cell_voltage.empty() ? nan : static_cast<float>(v_sum);
          // sensor_msgs/BatteryState: current is negative when discharging
          s.current = (b.current_battery == -1) ? nan : -b.current_battery * 0.01f;  // cA -> A
          s.charge = (b.current_consumed == -1) ? nan : -b.current_consumed * 1e-3f;  // mAh -> Ah (consumed)
          s.capacity = nan;
          s.design_capacity = nan;
          s.percentage = (b.battery_remaining == -1) ? nan : b.battery_remaining / 100.0f;
          s.temperature = (b.temperature == INT16_MAX) ? nan : b.temperature * 0.01f;
          s.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
          s.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
          s.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
          s.present = true;
          battery_pub_->publish(s);
          break;
        }
      case MAVLINK_MSG_ID_RC_CHANNELS: {
          mavlink_rc_channels_t r;
          mavlink_msg_rc_channels_decode(&m, &r);
          const uint16_t ch[18] = {
            r.chan1_raw, r.chan2_raw, r.chan3_raw, r.chan4_raw, r.chan5_raw, r.chan6_raw,
            r.chan7_raw, r.chan8_raw, r.chan9_raw, r.chan10_raw, r.chan11_raw, r.chan12_raw,
            r.chan13_raw, r.chan14_raw, r.chan15_raw, r.chan16_raw, r.chan17_raw, r.chan18_raw};
          mavros_msgs::msg::RCIn out;
          out.header = header();
          out.rssi = r.rssi;
          const size_t n = std::min<size_t>(r.chancount, 18);
          out.channels.assign(ch, ch + n);
          rc_pub_->publish(out);
          break;
        }
      case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW: {
          mavlink_servo_output_raw_t r;
          mavlink_msg_servo_output_raw_decode(&m, &r);
          const uint16_t ch[16] = {
            r.servo1_raw, r.servo2_raw, r.servo3_raw, r.servo4_raw, r.servo5_raw, r.servo6_raw,
            r.servo7_raw, r.servo8_raw, r.servo9_raw, r.servo10_raw, r.servo11_raw, r.servo12_raw,
            r.servo13_raw, r.servo14_raw, r.servo15_raw, r.servo16_raw};
          mavros_msgs::msg::RCOut out;
          out.header = header();
          out.channels.assign(ch, ch + 16);
          servo_pub_->publish(out);
          break;
        }
      case MAVLINK_MSG_ID_RAW_IMU: {
          // Raw (unscaled) sensor values: kept as-is, order = acc xyz, gyro xyz, mag xyz.
          mavlink_raw_imu_t r;
          mavlink_msg_raw_imu_decode(&m, &r);
//          std_msgs::msg::Int16MultiArray out;
//          out.data = {r.xacc, r.yacc, r.zacc, r.xgyro, r.ygyro, r.zgyro, r.xmag, r.ymag, r.zmag};
//          raw_imu_pub_->publish(out);
          mavlink_msg::msg::IMU imu_msg;
          auto h = header();
          imu_msg.header = h;
          imu_msg.time_usec = r.time_usec; // us
          imu_msg.xacc = r.xacc; // mG
          imu_msg.yacc = r.yacc; // mG
          imu_msg.zacc = r.zacc; // mG
          imu_msg.xgyro = r.xgyro; // mrad/s
          imu_msg.ygyro = r.ygyro; // mrad/s
          imu_msg.zgyro = r.zgyro; // mrad/s
          imu_msg.xmag = r.xmag; // mgauss
          imu_msg.ymag = r.ymag; // mgauss
          imu_msg.zmag = r.zmag; // mgauss
          imu_msg.id = r.id;
          rawImuPub_->publish(imu_msg);

          sensor_msgs::msg::Temperature t;
          t.header = h;
          t.temperature = r.temperature * 0.01;;
          rawImuTempPub_->publish(t);

          break;
        }
      case MAVLINK_MSG_ID_SCALED_IMU2: {
          mavlink_scaled_imu2_t s;
          mavlink_msg_scaled_imu2_decode(&m, &s);
          const auto h = header();
          sensor_msgs::msg::Imu imu;
          imu.header = h;
          imu.orientation_covariance[0] = -1.0;  // REP 145: no orientation provided
          imu.linear_acceleration.x = s.xacc * 1e-3 * kG;   // mG -> m/s^2
          imu.linear_acceleration.y = s.yacc * 1e-3 * kG;
          imu.linear_acceleration.z = s.zacc * 1e-3 * kG;
          imu.angular_velocity.x = s.xgyro * 1e-3;          // mrad/s -> rad/s
          imu.angular_velocity.y = s.ygyro * 1e-3;
          imu.angular_velocity.z = s.zgyro * 1e-3;
          imu2_pub_->publish(imu);
          sensor_msgs::msg::MagneticField mag;
          mag.header = h;
          mag.magnetic_field.x = s.xmag * 1e-7;             // mgauss -> tesla
          mag.magnetic_field.y = s.ymag * 1e-7;
          mag.magnetic_field.z = s.zmag * 1e-7;
          imu2_mag_pub_->publish(mag);
          sensor_msgs::msg::Temperature t;
          t.header = h;
          t.temperature = s.temperature * 0.01;
          imu2_temp_pub_->publish(t);
          break;
        }
      case MAVLINK_MSG_ID_SCALED_PRESSURE: {
          mavlink_scaled_pressure_t p;
          mavlink_msg_scaled_pressure_decode(&m, &p);
          publish_pressure(press_pub_, press_temp_pub_, p.press_abs, p.temperature);
          break;
        }
      case MAVLINK_MSG_ID_SCALED_PRESSURE2: {
          mavlink_scaled_pressure2_t p;
          mavlink_msg_scaled_pressure2_decode(&m, &p);
          publish_pressure(press2_pub_, press2_temp_pub_, p.press_abs, p.temperature);
          break;
        }
      default:
        break;
    }
  }

  std::string frame_id_;
  std::unique_ptr<cpp_mavlink::MavlinkUdp> link_;
  std::atomic<bool> running_{false};
  std::thread thread_;

  //rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr attitude_pub_, attitude_rate_pub_;
  rclcpp::Publisher<mavlink_msg::msg::Attitude>::SharedPtr attitudePub_;

  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  rclcpp::Publisher<mavros_msgs::msg::RCIn>::SharedPtr rc_pub_;
  rclcpp::Publisher<mavros_msgs::msg::RCOut>::SharedPtr servo_pub_;
  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr raw_imu_pub_;

  rclcpp::Publisher<mavlink_msg::msg::IMU>::SharedPtr rawImuPub_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr rawImuTempPub_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu2_pub_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr imu2_mag_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr imu2_temp_pub_, press_temp_pub_, press2_temp_pub_;
  rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr press_pub_, press2_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MavlinkPublisher>());
  rclcpp::shutdown();
  return 0;
}
