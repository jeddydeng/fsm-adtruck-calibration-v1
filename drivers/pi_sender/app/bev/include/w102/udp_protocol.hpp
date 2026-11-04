#pragma once

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <netinet/in.h>

namespace w102 {

constexpr std::uint8_t kProtocolVersion = 1;
constexpr std::size_t kDataHeaderSize = 56;
constexpr std::size_t kControlPacketSize = 24;
constexpr std::size_t kFragmentPayloadSize = 1400;

enum class MessageType : std::uint16_t {
  kCameraJpeg = 1,
  kBevJpeg = 2,
  kHealthJson = 3,
  kCoveragePng = 4,
  kControlJson = 5,
};

enum FrameFlags : std::uint16_t {
  kFlagJpeg = 1U << 0U,
  kFlagReplay = 1U << 1U,
  kFlagValid = 1U << 2U,
  kFlagPreviewOnly = 1U << 3U,
  kFlagSyntheticFill = 1U << 4U,
  kFlagRepeatedFrame = 1U << 5U,
  kFlagPartialCameras = 1U << 6U,
  kFlagIlluminationAdjusted = 1U << 7U,
};

enum class ControlCommand : std::uint8_t {
  kHello = 1,
  kSetFeatures = 2,
  kSetOutputFps = 3,
  kSetJpegQuality = 4,
  kCaptureRaw = 5,
  kSetOutputMode = 6,
};

enum FeatureBits : std::uint16_t {
  kFeatureRawCameras = 1U << 0U,
  kFeatureBev = 1U << 1U,
};

struct DataHeader {
  MessageType message_type{MessageType::kCameraJpeg};
  std::uint16_t flags{0};
  std::uint16_t stream_id{0};
  std::uint64_t boot_id{0};
  std::uint64_t frame_id{0};
  std::uint64_t capture_mono_ns{0};
  std::uint32_t total_size{0};
  std::uint32_t frame_crc32{0};
  std::uint16_t fragment_index{0};
  std::uint16_t fragment_count{0};
  std::uint16_t payload_size{0};
};

struct ControlPacket {
  ControlCommand command{ControlCommand::kHello};
  std::uint16_t stream_mask{0xFFFF};
  std::uint16_t reply_port{5600};
  std::uint64_t nonce{0};
};

std::uint32_t crc32(const std::uint8_t* data, std::size_t size);
std::vector<std::uint8_t> encode_data_header(const DataHeader& header);
std::optional<DataHeader> decode_data_header(const std::uint8_t* data,
                                             std::size_t size);
std::vector<std::uint8_t> encode_control_packet(const ControlPacket& packet);
std::optional<ControlPacket> decode_control_packet(const std::uint8_t* data,
                                                   std::size_t size);
std::uint64_t monotonic_time_ns();
std::uint64_t random_boot_id();

class UdpFragmentSender {
 public:
  UdpFragmentSender(std::string peer_ip, std::uint16_t peer_port,
                    std::size_t max_payload);
  ~UdpFragmentSender();

  UdpFragmentSender(const UdpFragmentSender&) = delete;
  UdpFragmentSender& operator=(const UdpFragmentSender&) = delete;

  void send_frame(const std::vector<std::uint8_t>& payload,
                  MessageType message_type, std::uint16_t flags,
                  std::uint16_t stream_id, std::uint64_t boot_id,
                  std::uint64_t frame_id, std::uint64_t capture_mono_ns);
  void send_frame_to(const std::vector<std::uint8_t>& payload,
                     MessageType message_type, std::uint16_t flags,
                     std::uint16_t stream_id, std::uint64_t boot_id,
                     std::uint64_t frame_id, std::uint64_t capture_mono_ns,
                     const sockaddr_in& destination);
  sockaddr_in peer_address() const;

 private:
  void send_impl(const std::vector<std::uint8_t>& payload,
                 MessageType message_type, std::uint16_t flags,
                 std::uint16_t stream_id, std::uint64_t boot_id,
                 std::uint64_t frame_id, std::uint64_t capture_mono_ns,
                 const sockaddr_in& destination);

  int socket_fd_{-1};
  sockaddr_in peer_{};
  std::size_t max_payload_{1400};
  std::mutex send_mutex_;
};

}  // namespace w102
