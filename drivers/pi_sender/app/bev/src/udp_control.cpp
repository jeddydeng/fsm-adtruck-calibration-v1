#include "w102/udp_protocol.hpp"

#include <arpa/inet.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

struct Options {
  std::string gateway_ip{"192.168.1.188"};
  std::uint16_t control_port{5601};
  std::uint16_t reply_port{5602};
  w102::ControlCommand command{w102::ControlCommand::kHello};
  std::uint16_t value{0xFFFF};
};

std::uint16_t port_value(const char* text) {
  const int value = std::stoi(text);
  if (value < 1 || value > 65535) throw std::invalid_argument("invalid port");
  return static_cast<std::uint16_t>(value);
}

std::uint16_t uint16_value(const char* text) {
  const auto value = std::stoul(text, nullptr, 0);
  if (value > 65535U) throw std::invalid_argument("value exceeds uint16");
  return static_cast<std::uint16_t>(value);
}

Options parse_options(int argc, char** argv) {
  Options options;
  bool command_set = false;
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    auto next = [&]() -> const char* {
      if (++index >= argc) throw std::invalid_argument("missing option value");
      return argv[index];
    };
    if (argument == "--gateway-ip") {
      options.gateway_ip = next();
    } else if (argument == "--control-port") {
      options.control_port = port_value(next());
    } else if (argument == "--reply-port") {
      options.reply_port = port_value(next());
    } else if (argument == "--features") {
      options.command = w102::ControlCommand::kSetFeatures;
      options.value = uint16_value(next());
      command_set = true;
    } else if (argument == "--fps") {
      options.command = w102::ControlCommand::kSetOutputFps;
      options.value = uint16_value(next());
      command_set = true;
    } else if (argument == "--jpeg-quality") {
      options.command = w102::ControlCommand::kSetJpegQuality;
      options.value = uint16_value(next());
      command_set = true;
    } else if (argument == "--help") {
      std::cout << "w102_udp_control options:\n"
                << "  --gateway-ip IPv4 --control-port PORT --reply-port PORT\n"
                << "  exactly one of: --features MASK | --fps N (10-30) | "
                   "--jpeg-quality N\n"
                << "  feature mask: bit0=raw cameras, bit1=BEV\n";
      std::exit(0);
    } else {
      throw std::invalid_argument("unknown option: " + argument);
    }
  }
  if (!command_set) throw std::invalid_argument("a set command is required");
  return options;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Options options = parse_options(argc, argv);
    const int socket_fd = socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
    if (socket_fd < 0) throw std::runtime_error("socket failed");
    sockaddr_in local{};
    local.sin_family = AF_INET;
    local.sin_port = htons(options.reply_port);
    local.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(socket_fd, reinterpret_cast<const sockaddr*>(&local),
             sizeof(local)) != 0) {
      close(socket_fd);
      throw std::runtime_error("cannot bind reply port");
    }
    sockaddr_in gateway{};
    gateway.sin_family = AF_INET;
    gateway.sin_port = htons(options.control_port);
    if (inet_pton(AF_INET, options.gateway_ip.c_str(), &gateway.sin_addr) != 1) {
      close(socket_fd);
      throw std::invalid_argument("invalid gateway address");
    }
    w102::ControlPacket request;
    request.command = options.command;
    request.stream_mask = options.value;
    request.reply_port = options.reply_port;
    request.nonce = w102::random_boot_id();
    const auto encoded = w102::encode_control_packet(request);
    if (sendto(socket_fd, encoded.data(), encoded.size(), 0,
               reinterpret_cast<const sockaddr*>(&gateway), sizeof(gateway)) <
        0) {
      close(socket_fd);
      throw std::runtime_error("send failed");
    }
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::seconds(2);
    std::array<std::uint8_t, 2048> datagram{};
    while (std::chrono::steady_clock::now() < deadline) {
      pollfd descriptor{socket_fd, POLLIN, 0};
      if (poll(&descriptor, 1, 200) <= 0) continue;
      sockaddr_in source{};
      socklen_t source_length = sizeof(source);
      const auto received = recvfrom(
          socket_fd, datagram.data(), datagram.size(), 0,
          reinterpret_cast<sockaddr*>(&source), &source_length);
      if (received <= 0 || source.sin_addr.s_addr != gateway.sin_addr.s_addr) {
        continue;
      }
      const auto header = w102::decode_data_header(datagram.data(), received);
      if (!header || header->message_type != w102::MessageType::kControlJson ||
          header->fragment_count != 1 ||
          header->payload_size != header->total_size) {
        continue;
      }
      const auto* payload = datagram.data() + w102::kDataHeaderSize;
      if (w102::crc32(payload, header->payload_size) != header->frame_crc32) {
        continue;
      }
      const std::string response(reinterpret_cast<const char*>(payload),
                                 header->payload_size);
      const std::string expected_nonce =
          "\"nonce\":" + std::to_string(request.nonce);
      if (response.find(expected_nonce) == std::string::npos) continue;
      std::cout.write(reinterpret_cast<const char*>(payload),
                      header->payload_size);
      std::cout << '\n';
      close(socket_fd);
      return (header->flags & w102::kFlagValid) != 0 ? 0 : 2;
    }
    close(socket_fd);
    std::cerr << "control acknowledgement timeout\n";
    return 3;
  } catch (const std::exception& error) {
    std::cerr << "fatal: " << error.what() << '\n';
    return 1;
  }
}
