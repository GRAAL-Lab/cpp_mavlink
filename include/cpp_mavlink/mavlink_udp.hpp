// Minimal blocking UDP transport for MAVLink (equivalent of pymavlink's
// "udpin:" and "udp:"/"udpout:" connection strings).
#pragma once

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <cstdint>
#include <stdexcept>
#include <string>

#include <mavlink/v2.0/common/mavlink.h>

namespace cpp_mavlink
{

class MavlinkUdp
{
public:
  // kListen  : bind host:port and wait for the vehicle to talk to us (pymavlink "udpin:")
  // kConnect : send to host:port; replies come back to our ephemeral port (pymavlink "udp:")
  enum class Mode { kListen, kConnect };

  MavlinkUdp(Mode mode, const std::string & host, uint16_t port,
    int timeout_ms = 100, uint8_t channel = MAVLINK_COMM_0)
  : mode_(mode), channel_(channel)
  {
    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0) {
      throw std::runtime_error("MavlinkUdp: socket() failed");
    }
    timeval tv{timeout_ms / 1000, (timeout_ms % 1000) * 1000};
    ::setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    if (::inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1) {
      ::close(fd_);
      throw std::runtime_error("MavlinkUdp: invalid IPv4 address: " + host);
    }
    if (mode == Mode::kListen) {
      if (::bind(fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
        ::close(fd_);
        throw std::runtime_error("MavlinkUdp: bind() failed on " + host);
      }
    } else {
      peer_ = addr;
      has_peer_ = true;
    }
  }

  ~MavlinkUdp() {if (fd_ >= 0) {::close(fd_);}}
  MavlinkUdp(const MavlinkUdp &) = delete;
  MavlinkUdp & operator=(const MavlinkUdp &) = delete;

  // Returns true when a complete, CRC-valid message was parsed.
  // Returns false on receive timeout (caller should re-check its run flag).
  bool receive(mavlink_message_t & msg)
  {
    for (;;) {
      while (pos_ < len_) {
        if (mavlink_parse_char(channel_, buf_[pos_++], &msg, &status_)) {
          return true;
        }
      }
      sockaddr_in src{};
      socklen_t sl = sizeof(src);
      ssize_t n = ::recvfrom(fd_, buf_, sizeof(buf_), 0,
          reinterpret_cast<sockaddr *>(&src), &sl);
      if (n <= 0) {
        return false;
      }
      pos_ = 0;
      len_ = static_cast<size_t>(n);
      if (mode_ == Mode::kListen) {
        peer_ = src;  // remember who to answer
        has_peer_ = true;
      }
    }
  }

  bool send(const mavlink_message_t & msg)
  {
    if (!has_peer_) {return false;}
    uint8_t out[MAVLINK_MAX_PACKET_LEN];
    const uint16_t n = mavlink_msg_to_send_buffer(out, &msg);
    return ::sendto(fd_, out, n, 0,
             reinterpret_cast<const sockaddr *>(&peer_), sizeof(peer_)) == n;
  }

private:
  Mode mode_;
  uint8_t channel_;
  int fd_{-1};
  sockaddr_in peer_{};
  bool has_peer_{false};
  mavlink_status_t status_{};
  uint8_t buf_[2048];
  size_t pos_{0}, len_{0};
};

}  // namespace cpp_mavlink
