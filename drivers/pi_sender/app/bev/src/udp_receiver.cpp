#include "w102/udp_protocol.hpp"

#include <arpa/inet.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstring>
#include <fstream>
#include <iostream>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <tuple>
#include <unordered_set>
#include <vector>

namespace {

struct Options {
  std::string gateway_ip{"192.168.1.188"};
  std::uint16_t data_port{5600};
  std::uint16_t control_port{5601};
  std::string output{"/tmp/w102-latest.jpg"};
  int timeout_seconds{10};
  int frame_count{1};
};

Options parse_options(int argc, char** argv) {
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    auto next = [&]() -> const char* {
      if (++index >= argc) throw std::invalid_argument("missing option value");
      return argv[index];
    };
    if (argument == "--gateway-ip") options.gateway_ip = next();
    else if (argument == "--data-port") options.data_port = std::stoi(next());
    else if (argument == "--control-port") options.control_port = std::stoi(next());
    else if (argument == "--output") options.output = next();
    else if (argument == "--timeout") options.timeout_seconds = std::stoi(next());
    else if (argument == "--frames") options.frame_count = std::stoi(next());
    else throw std::invalid_argument("unknown option: " + argument);
  }
  return options;
}

struct AssemblyKey {
  std::uint64_t boot_id{0};
  std::uint64_t frame_id{0};
  std::uint16_t message_type{0};
  std::uint16_t stream_id{0};
  bool operator<(const AssemblyKey& other) const {
    return std::tie(boot_id, frame_id, message_type, stream_id) <
           std::tie(other.boot_id, other.frame_id, other.message_type,
                    other.stream_id);
  }
};

struct Assembly {
  w102::DataHeader first_header{};
  std::vector<std::vector<std::uint8_t>> fragments;
  std::vector<bool> received;
  std::size_t bytes{0};
  std::chrono::steady_clock::time_point created{std::chrono::steady_clock::now()};
};

void write_atomic(const std::string& path,
                  const std::vector<std::uint8_t>& payload) {
  const std::string temporary = path + ".tmp";
  {
    std::ofstream output(temporary, std::ios::binary | std::ios::trunc);
    output.write(reinterpret_cast<const char*>(payload.data()), payload.size());
    if (!output) throw std::runtime_error("failed to write " + temporary);
  }
  if (rename(temporary.c_str(), path.c_str()) != 0) {
    throw std::runtime_error("failed to rename receiver output");
  }
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Options options = parse_options(argc, argv);
    const int socket_fd = socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
    if (socket_fd < 0) throw std::runtime_error("socket failed");
    int receive_buffer = 8 * 1024 * 1024;
    setsockopt(socket_fd, SOL_SOCKET, SO_RCVBUF, &receive_buffer,
               sizeof(receive_buffer));
    sockaddr_in local{};
    local.sin_family = AF_INET;
    local.sin_port = htons(options.data_port);
    local.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(socket_fd, reinterpret_cast<const sockaddr*>(&local),
             sizeof(local)) != 0) {
      throw std::runtime_error("bind failed");
    }

    const int control_fd = socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
    sockaddr_in gateway{};
    gateway.sin_family = AF_INET;
    gateway.sin_port = htons(options.control_port);
    if (inet_pton(AF_INET, options.gateway_ip.c_str(), &gateway.sin_addr) != 1) {
      throw std::invalid_argument("invalid gateway address");
    }
    w102::ControlPacket hello;
    hello.command = w102::ControlCommand::kHello;
    hello.reply_port = options.data_port;
    hello.nonce = w102::random_boot_id();
    const auto hello_bytes = w102::encode_control_packet(hello);
    sendto(control_fd, hello_bytes.data(), hello_bytes.size(), 0,
           reinterpret_cast<const sockaddr*>(&gateway), sizeof(gateway));
    close(control_fd);

    std::map<AssemblyKey, Assembly> assemblies;
    std::map<std::pair<std::uint16_t, std::uint16_t>, std::uint64_t>
        last_completed;
    std::unordered_set<std::uint64_t> retired_boot_ids;
    std::uint64_t active_boot_id = 0;
    std::array<std::uint8_t, 2048> datagram{};
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::seconds(options.timeout_seconds);
    int completed_images = 0;
    while (std::chrono::steady_clock::now() < deadline &&
           completed_images < options.frame_count) {
      const auto cleanup_now = std::chrono::steady_clock::now();
      for (auto iterator = assemblies.begin(); iterator != assemblies.end();) {
        if (cleanup_now - iterator->second.created >
            std::chrono::milliseconds(300)) {
          iterator = assemblies.erase(iterator);
        } else {
          ++iterator;
        }
      }
      if (assemblies.size() > 256) assemblies.clear();
      pollfd descriptor{socket_fd, POLLIN, 0};
      if (poll(&descriptor, 1, 200) <= 0) continue;
      sockaddr_in source{};
      socklen_t source_length = sizeof(source);
      const auto received = recvfrom(
          socket_fd, datagram.data(), datagram.size(), 0,
          reinterpret_cast<sockaddr*>(&source), &source_length);
      if (received <= 0) continue;
      if (source.sin_family != AF_INET ||
          source.sin_addr.s_addr != gateway.sin_addr.s_addr) {
        continue;
      }
      const auto header =
          w102::decode_data_header(datagram.data(), received);
      if (!header || header->total_size == 0 ||
          header->total_size > 16U * 1024U * 1024U) {
        continue;
      }
      const auto expected_count = static_cast<std::uint16_t>(
          (header->total_size + w102::kFragmentPayloadSize - 1) /
          w102::kFragmentPayloadSize);
      const auto fragment_offset =
          static_cast<std::size_t>(header->fragment_index) *
          w102::kFragmentPayloadSize;
      if (fragment_offset >= header->total_size) continue;
      const auto expected_payload = std::min<std::size_t>(
          w102::kFragmentPayloadSize, header->total_size - fragment_offset);
      if (header->fragment_count != expected_count ||
          header->payload_size != expected_payload) {
        continue;
      }
      if (retired_boot_ids.count(header->boot_id) != 0) continue;
      if (active_boot_id == 0) active_boot_id = header->boot_id;
      if (header->boot_id != active_boot_id) {
        retired_boot_ids.insert(active_boot_id);
        if (retired_boot_ids.size() > 8) retired_boot_ids.clear();
        active_boot_id = header->boot_id;
        assemblies.clear();
        last_completed.clear();
      }
      AssemblyKey key{header->boot_id, header->frame_id,
                      static_cast<std::uint16_t>(header->message_type),
                      header->stream_id};
      auto& assembly = assemblies[key];
      if (assembly.fragments.empty()) {
        assembly.first_header = *header;
        assembly.fragments.resize(header->fragment_count);
        assembly.received.resize(header->fragment_count, false);
      }
      if (assembly.fragments.size() != header->fragment_count ||
          assembly.first_header.total_size != header->total_size ||
          assembly.first_header.frame_crc32 != header->frame_crc32 ||
          assembly.first_header.flags != header->flags ||
          assembly.first_header.capture_mono_ns != header->capture_mono_ns) {
        assemblies.erase(key);
        continue;
      }
      if (!assembly.received[header->fragment_index]) {
        const auto begin = datagram.begin() + w102::kDataHeaderSize;
        assembly.fragments[header->fragment_index] =
            std::vector<std::uint8_t>(begin, begin + header->payload_size);
        assembly.received[header->fragment_index] = true;
        assembly.bytes += header->payload_size;
      }
      bool complete = assembly.bytes == assembly.first_header.total_size;
      for (const bool present : assembly.received) complete = complete && present;
      if (!complete) continue;

      std::vector<std::uint8_t> payload;
      payload.reserve(assembly.first_header.total_size);
      for (const auto& fragment : assembly.fragments) {
        payload.insert(payload.end(), fragment.begin(), fragment.end());
      }
      if (payload.size() != assembly.first_header.total_size ||
          w102::crc32(payload.data(), payload.size()) !=
              assembly.first_header.frame_crc32) {
        assemblies.erase(key);
        continue;
      }
      const auto stream_key = std::make_pair(
          static_cast<std::uint16_t>(assembly.first_header.message_type),
          assembly.first_header.stream_id);
      const auto previous = last_completed.find(stream_key);
      if (previous != last_completed.end() && key.frame_id <= previous->second) {
        assemblies.erase(key);
        continue;
      }
      last_completed[stream_key] = key.frame_id;
      if (assembly.first_header.message_type ==
              w102::MessageType::kCameraJpeg ||
          assembly.first_header.message_type == w102::MessageType::kBevJpeg) {
        write_atomic(options.output, payload);
        ++completed_images;
        std::cout << "complete image frame=" << key.frame_id
                  << " bytes=" << payload.size()
                  << " flags=" << assembly.first_header.flags << '\n';
      } else if (assembly.first_header.message_type ==
                 w102::MessageType::kHealthJson) {
        std::cout << "health: " << std::string(payload.begin(), payload.end())
                  << '\n';
      } else if (assembly.first_header.message_type ==
                 w102::MessageType::kControlJson) {
        std::cout << "control: " << std::string(payload.begin(), payload.end())
                  << '\n';
      }
      assemblies.erase(key);

    }
    close(socket_fd);
    return completed_images == options.frame_count ? 0 : 2;
  } catch (const std::exception& error) {
    std::cerr << "fatal: " << error.what() << '\n';
    return 1;
  }
}
