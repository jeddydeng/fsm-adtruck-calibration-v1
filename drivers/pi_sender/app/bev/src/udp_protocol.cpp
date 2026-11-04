#include "w102/udp_protocol.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cstring>
#include <random>
#include <stdexcept>

namespace w102 {
namespace {

void append_u16(std::vector<std::uint8_t>& output, std::uint16_t value) {
  output.push_back(static_cast<std::uint8_t>((value >> 8U) & 0xFFU));
  output.push_back(static_cast<std::uint8_t>(value & 0xFFU));
}

void append_u32(std::vector<std::uint8_t>& output, std::uint32_t value) {
  output.push_back(static_cast<std::uint8_t>((value >> 24U) & 0xFFU));
  output.push_back(static_cast<std::uint8_t>((value >> 16U) & 0xFFU));
  output.push_back(static_cast<std::uint8_t>((value >> 8U) & 0xFFU));
  output.push_back(static_cast<std::uint8_t>(value & 0xFFU));
}

void append_u64(std::vector<std::uint8_t>& output, std::uint64_t value) {
  for (int shift = 56; shift >= 0; shift -= 8) {
    output.push_back(static_cast<std::uint8_t>((value >> shift) & 0xFFU));
  }
}

std::uint16_t read_u16(const std::uint8_t* data) {
  return static_cast<std::uint16_t>(
      (static_cast<std::uint16_t>(data[0]) << 8U) |
      static_cast<std::uint16_t>(data[1]));
}

std::uint32_t read_u32(const std::uint8_t* data) {
  return (static_cast<std::uint32_t>(data[0]) << 24U) |
         (static_cast<std::uint32_t>(data[1]) << 16U) |
         (static_cast<std::uint32_t>(data[2]) << 8U) |
         static_cast<std::uint32_t>(data[3]);
}

std::uint64_t read_u64(const std::uint8_t* data) {
  std::uint64_t value = 0;
  for (int index = 0; index < 8; ++index) {
    value = (value << 8U) | data[index];
  }
  return value;
}

}  // namespace

std::uint32_t crc32(const std::uint8_t* data, std::size_t size) {
  std::uint32_t crc = 0xFFFFFFFFU;
  for (std::size_t index = 0; index < size; ++index) {
    crc ^= data[index];
    for (int bit = 0; bit < 8; ++bit) {
      const std::uint32_t mask =
          static_cast<std::uint32_t>(-(static_cast<std::int32_t>(crc & 1U)));
      crc = (crc >> 1U) ^ (0xEDB88320U & mask);
    }
  }
  return ~crc;
}

std::vector<std::uint8_t> encode_data_header(const DataHeader& header) {
  std::vector<std::uint8_t> output;
  output.reserve(kDataHeaderSize);
  output.insert(output.end(), {'W', '1', 'U', 'P'});
  output.push_back(kProtocolVersion);
  output.push_back(static_cast<std::uint8_t>(kDataHeaderSize));
  append_u16(output, static_cast<std::uint16_t>(header.message_type));
  append_u16(output, header.flags);
  append_u16(output, header.stream_id);
  append_u64(output, header.boot_id);
  append_u64(output, header.frame_id);
  append_u64(output, header.capture_mono_ns);
  append_u32(output, header.total_size);
  append_u32(output, header.frame_crc32);
  append_u16(output, header.fragment_index);
  append_u16(output, header.fragment_count);
  append_u16(output, header.payload_size);
  append_u16(output, 0);
  append_u32(output, crc32(output.data(), output.size()));
  if (output.size() != kDataHeaderSize) {
    throw std::logic_error("invalid encoded data header size");
  }
  return output;
}

std::optional<DataHeader> decode_data_header(const std::uint8_t* data,
                                             std::size_t size) {
  if (size < kDataHeaderSize || std::memcmp(data, "W1UP", 4) != 0 ||
      data[4] != kProtocolVersion || data[5] != kDataHeaderSize) {
    return std::nullopt;
  }
  if (crc32(data, 52) != read_u32(data + 52)) {
    return std::nullopt;
  }
  DataHeader header;
  header.message_type = static_cast<MessageType>(read_u16(data + 6));
  header.flags = read_u16(data + 8);
  header.stream_id = read_u16(data + 10);
  header.boot_id = read_u64(data + 12);
  header.frame_id = read_u64(data + 20);
  header.capture_mono_ns = read_u64(data + 28);
  header.total_size = read_u32(data + 36);
  header.frame_crc32 = read_u32(data + 40);
  header.fragment_index = read_u16(data + 44);
  header.fragment_count = read_u16(data + 46);
  header.payload_size = read_u16(data + 48);
  if (header.fragment_count == 0 ||
      header.fragment_index >= header.fragment_count ||
      kDataHeaderSize + header.payload_size > size) {
    return std::nullopt;
  }
  return header;
}

std::vector<std::uint8_t> encode_control_packet(const ControlPacket& packet) {
  std::vector<std::uint8_t> output;
  output.reserve(kControlPacketSize);
  output.insert(output.end(), {'W', '1', 'C', 'T'});
  output.push_back(kProtocolVersion);
  output.push_back(static_cast<std::uint8_t>(packet.command));
  append_u16(output, packet.stream_mask);
  append_u16(output, packet.reply_port);
  append_u16(output, 0);
  append_u64(output, packet.nonce);
  append_u32(output, crc32(output.data(), output.size()));
  return output;
}

std::optional<ControlPacket> decode_control_packet(const std::uint8_t* data,
                                                   std::size_t size) {
  if (size != kControlPacketSize || std::memcmp(data, "W1CT", 4) != 0 ||
      data[4] != kProtocolVersion || crc32(data, 20) != read_u32(data + 20)) {
    return std::nullopt;
  }
  ControlPacket packet;
  packet.command = static_cast<ControlCommand>(data[5]);
  packet.stream_mask = read_u16(data + 6);
  packet.reply_port = read_u16(data + 8);
  packet.nonce = read_u64(data + 12);
  return packet;
}

std::uint64_t monotonic_time_ns() {
  timespec value{};
  if (clock_gettime(CLOCK_MONOTONIC, &value) != 0) {
    throw std::runtime_error("clock_gettime(CLOCK_MONOTONIC) failed");
  }
  return static_cast<std::uint64_t>(value.tv_sec) * 1000000000ULL +
         static_cast<std::uint64_t>(value.tv_nsec);
}

std::uint64_t random_boot_id() {
  std::random_device random_device;
  std::mt19937_64 generator(random_device());
  return generator();
}

UdpFragmentSender::UdpFragmentSender(std::string peer_ip,
                                     std::uint16_t peer_port,
                                     std::size_t max_payload)
    : max_payload_(max_payload) {
  if (max_payload_ < 256 || max_payload_ > 1400) {
    throw std::invalid_argument("max_payload must be in [256, 1400]");
  }
  socket_fd_ = socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
  if (socket_fd_ < 0) {
    throw std::runtime_error("failed to create UDP socket");
  }
  int send_buffer = 4 * 1024 * 1024;
  setsockopt(socket_fd_, SOL_SOCKET, SO_SNDBUF, &send_buffer,
             sizeof(send_buffer));
  peer_.sin_family = AF_INET;
  peer_.sin_port = htons(peer_port);
  if (inet_pton(AF_INET, peer_ip.c_str(), &peer_.sin_addr) != 1) {
    close(socket_fd_);
    socket_fd_ = -1;
    throw std::invalid_argument("invalid IPv4 peer address: " + peer_ip);
  }
}

UdpFragmentSender::~UdpFragmentSender() {
  if (socket_fd_ >= 0) {
    close(socket_fd_);
  }
}

sockaddr_in UdpFragmentSender::peer_address() const { return peer_; }

void UdpFragmentSender::send_frame(
    const std::vector<std::uint8_t>& payload, MessageType message_type,
    std::uint16_t flags, std::uint16_t stream_id, std::uint64_t boot_id,
    std::uint64_t frame_id, std::uint64_t capture_mono_ns) {
  send_impl(payload, message_type, flags, stream_id, boot_id, frame_id,
            capture_mono_ns, peer_);
}

void UdpFragmentSender::send_frame_to(
    const std::vector<std::uint8_t>& payload, MessageType message_type,
    std::uint16_t flags, std::uint16_t stream_id, std::uint64_t boot_id,
    std::uint64_t frame_id, std::uint64_t capture_mono_ns,
    const sockaddr_in& destination) {
  send_impl(payload, message_type, flags, stream_id, boot_id, frame_id,
            capture_mono_ns, destination);
}

void UdpFragmentSender::send_impl(
    const std::vector<std::uint8_t>& payload, MessageType message_type,
    std::uint16_t flags, std::uint16_t stream_id, std::uint64_t boot_id,
    std::uint64_t frame_id, std::uint64_t capture_mono_ns,
    const sockaddr_in& destination) {
  std::lock_guard<std::mutex> lock(send_mutex_);
  if (payload.empty() || payload.size() > 64U * 1024U * 1024U) {
    throw std::invalid_argument("payload is empty or too large");
  }
  const std::size_t count = (payload.size() + max_payload_ - 1) / max_payload_;
  if (count > 65535U) {
    throw std::invalid_argument("too many UDP fragments");
  }
  const auto payload_crc = crc32(payload.data(), payload.size());
  for (std::size_t fragment = 0; fragment < count; ++fragment) {
    const std::size_t offset = fragment * max_payload_;
    const std::size_t fragment_size =
        std::min(max_payload_, payload.size() - offset);
    DataHeader header;
    header.message_type = message_type;
    header.flags = flags;
    header.stream_id = stream_id;
    header.boot_id = boot_id;
    header.frame_id = frame_id;
    header.capture_mono_ns = capture_mono_ns;
    header.total_size = static_cast<std::uint32_t>(payload.size());
    header.frame_crc32 = payload_crc;
    header.fragment_index = static_cast<std::uint16_t>(fragment);
    header.fragment_count = static_cast<std::uint16_t>(count);
    header.payload_size = static_cast<std::uint16_t>(fragment_size);
    auto datagram = encode_data_header(header);
    datagram.insert(datagram.end(), payload.begin() + offset,
                    payload.begin() + offset + fragment_size);
    const auto sent = sendto(socket_fd_, datagram.data(), datagram.size(),
                             MSG_DONTWAIT,
                             reinterpret_cast<const sockaddr*>(&destination),
                             sizeof(destination));
    if (sent < 0) {
      // UDP has no persistent connection. A transient send failure is handled by
      // the next frame; callers keep the latest complete frame for HELLO replay.
      continue;
    }
  }
}

}  // namespace w102
