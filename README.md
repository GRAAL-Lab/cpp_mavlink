# cpp_mavlink

A ROS 2 C++ package for receiving MAVLink telemetry over UDP and publishing decoded data as ROS messages.

## Overview

`cpp_mavlink` is a C++ port of the Python-based MAVLink publisher workflow. 
- It listens for MAVLink packets on a UDP socket,
- decodes selected messages,
- and republishes them to ROS 2 topics for downstream processing, visualization, and control.

The package currently supports messages such as:

- attitude
- battery status
- RC input/output
- raw IMU
- scaled IMU
- pressure and temperature
- servo output

## Features

- MAVLink UDP listener
- ROS 2 integration using `rclcpp`
- Standard ROS message publishing
- Configurable bind address, port, and topic prefix
- Designed for MAVLink-based vehicle telemetry consumption

## Dependencies
This package depends on:

- ROS 2
- `rclcpp`
- `std_msgs`
- `geometry_msgs`
- `sensor_msgs`
- `mavros_msgs`
- `mavlink`
- `OpenCV`
- `mavlink_msg` (when custom MAVLink message definitions are used)

## Build

From a ROS 2 workspace:

```bash
colcon build --packages-select cpp_mavlink
```
Then source the workspace:
```bash
source install/setup.bash
```

## Run

ros2 run cpp_mavlink mavlink_publisher --ros-args \
  -p bind_address:=0.0.0.0 \
  -p port:=14551 \
  -p topic_prefix:=/bluerov2

Default settings:

  -bind address: 0.0.0.0
  -port: 14551
  -topic prefix: /bluerov2

