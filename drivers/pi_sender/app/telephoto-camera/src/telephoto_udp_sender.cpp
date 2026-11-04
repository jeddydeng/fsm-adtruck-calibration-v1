#include <arpa/inet.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <netinet/ip.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

constexpr char kMagic[4] = {'T', 'J', 'P', 'G'};
constexpr std::uint8_t kVersion = 1;
constexpr std::size_t kPayloadBytes = 1160;

#pragma pack(push, 1)
struct WireHeader {
  char magic[4];
  std::uint8_t version;
  std::uint8_t flags;
  std::uint16_t header_bytes;
  std::uint32_t frame_id;
  std::uint64_t capture_monotonic_ns;
  std::uint32_t frame_size;
  std::uint32_t crc32;
  std::uint16_t width;
  std::uint16_t height;
  std::uint16_t fragment_index;
  std::uint16_t fragment_count;
  std::uint16_t payload_size;
  std::uint16_t reserved;
};
#pragma pack(pop)

static_assert(sizeof(WireHeader) == 40, "unexpected telephoto wire header size");

struct Options {
  std::string device = "/dev/v4l/by-id/usb-LRCP_H960P_LRCP_H960P-video-index0";
  std::string peer_ip = "192.168.1.102";
  std::uint16_t port = 5620;
  std::uint16_t width = 320;
  std::uint16_t height = 240;
  std::uint32_t fps = 5;
};

struct MappedBuffer {
  void* address = MAP_FAILED;
  std::size_t length = 0;
};

std::atomic<bool> g_running{true};

void handle_signal(int) { g_running.store(false); }

std::uint64_t host_to_network_64(std::uint64_t value) {
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
  return (static_cast<std::uint64_t>(htonl(static_cast<std::uint32_t>(value))) << 32U) |
         htonl(static_cast<std::uint32_t>(value >> 32U));
#else
  return value;
#endif
}

int xioctl(int fd, unsigned long request, void* argument) {
  int result;
  do {
    result = ioctl(fd, request, argument);
  } while (result == -1 && errno == EINTR);
  return result;
}

std::uint32_t crc32(const std::uint8_t* data, std::size_t size) {
  std::uint32_t crc = 0xFFFFFFFFU;
  for (std::size_t i = 0; i < size; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      const std::uint32_t mask = 0U - (crc & 1U);
      crc = (crc >> 1U) ^ (0xEDB88320U & mask);
    }
  }
  return ~crc;
}

std::uint64_t monotonic_nanoseconds() {
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

Options parse_options(int argc, char** argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string argument = argv[i];
    auto value = [&]() -> std::string {
      if (++i >= argc) throw std::runtime_error("missing value for " + argument);
      return argv[i];
    };
    if (argument == "--device") options.device = value();
    else if (argument == "--peer-ip") options.peer_ip = value();
    else if (argument == "--port") options.port = static_cast<std::uint16_t>(std::stoul(value()));
    else if (argument == "--width") options.width = static_cast<std::uint16_t>(std::stoul(value()));
    else if (argument == "--height") options.height = static_cast<std::uint16_t>(std::stoul(value()));
    else if (argument == "--fps") options.fps = std::stoul(value());
    else if (argument == "--help") {
      std::cout << "telephoto_udp_sender [--device PATH] [--peer-ip IP] [--port PORT] "
                   "[--width N] [--height N] [--fps N]\n";
      std::exit(0);
    } else {
      throw std::runtime_error("unknown argument: " + argument);
    }
  }
  if (options.fps == 0 || options.width == 0 || options.height == 0) {
    throw std::runtime_error("width, height and fps must be non-zero");
  }
  return options;
}

class CameraStream {
 public:
  explicit CameraStream(const Options& options) : options_(options) {
    camera_fd_ = open(options.device.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
    if (camera_fd_ < 0) throw std::runtime_error("open camera failed: " + std::string(strerror(errno)));

    v4l2_capability capability{};
    if (xioctl(camera_fd_, VIDIOC_QUERYCAP, &capability) < 0 ||
        !(capability.device_caps & V4L2_CAP_VIDEO_CAPTURE) ||
        !(capability.device_caps & V4L2_CAP_STREAMING)) {
      throw std::runtime_error("device is not a streaming V4L2 capture device");
    }

    v4l2_format format{};
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    format.fmt.pix.width = options.width;
    format.fmt.pix.height = options.height;
    format.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
    format.fmt.pix.field = V4L2_FIELD_NONE;
    if (xioctl(camera_fd_, VIDIOC_S_FMT, &format) < 0) {
      throw std::runtime_error("VIDIOC_S_FMT failed: " + std::string(strerror(errno)));
    }
    if (format.fmt.pix.width != options.width || format.fmt.pix.height != options.height ||
        format.fmt.pix.pixelformat != V4L2_PIX_FMT_MJPEG) {
      throw std::runtime_error("camera did not accept requested MJPEG format");
    }

    v4l2_streamparm parameters{};
    parameters.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    parameters.parm.capture.timeperframe.numerator = 1;
    parameters.parm.capture.timeperframe.denominator = options.fps;
    if (xioctl(camera_fd_, VIDIOC_S_PARM, &parameters) < 0) {
      throw std::runtime_error("VIDIOC_S_PARM failed: " + std::string(strerror(errno)));
    }

    // This camera enables dynamic frame-rate exposure by default. In darkness it can
    // silently fall from 5 Hz to about 0.5 Hz. Disable that behaviour so the requested
    // cadence remains deterministic; automatic exposure itself stays enabled.
    v4l2_control exposure_priority{};
    exposure_priority.id = V4L2_CID_EXPOSURE_AUTO_PRIORITY;
    exposure_priority.value = 0;
    if (xioctl(camera_fd_, VIDIOC_S_CTRL, &exposure_priority) < 0) {
      std::cerr << "warning: could not disable dynamic exposure frame-rate: "
                << strerror(errno) << std::endl;
    }

    v4l2_requestbuffers request{};
    request.count = 4;
    request.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    request.memory = V4L2_MEMORY_MMAP;
    if (xioctl(camera_fd_, VIDIOC_REQBUFS, &request) < 0 || request.count < 2) {
      throw std::runtime_error("V4L2 buffer allocation failed");
    }

    buffers_.resize(request.count);
    for (std::uint32_t index = 0; index < request.count; ++index) {
      v4l2_buffer buffer{};
      buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buffer.memory = V4L2_MEMORY_MMAP;
      buffer.index = index;
      if (xioctl(camera_fd_, VIDIOC_QUERYBUF, &buffer) < 0) {
        throw std::runtime_error("VIDIOC_QUERYBUF failed");
      }
      buffers_[index].length = buffer.length;
      buffers_[index].address = mmap(nullptr, buffer.length, PROT_READ | PROT_WRITE, MAP_SHARED,
                                     camera_fd_, buffer.m.offset);
      if (buffers_[index].address == MAP_FAILED) throw std::runtime_error("mmap failed");
      if (xioctl(camera_fd_, VIDIOC_QBUF, &buffer) < 0) throw std::runtime_error("VIDIOC_QBUF failed");
    }

    auto type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(camera_fd_, VIDIOC_STREAMON, &type) < 0) {
      throw std::runtime_error("VIDIOC_STREAMON failed: " + std::string(strerror(errno)));
    }
    streaming_ = true;
  }

  ~CameraStream() {
    if (camera_fd_ >= 0 && streaming_) {
      auto type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      xioctl(camera_fd_, VIDIOC_STREAMOFF, &type);
    }
    for (const auto& buffer : buffers_) {
      if (buffer.address != MAP_FAILED) munmap(buffer.address, buffer.length);
    }
    if (camera_fd_ >= 0) close(camera_fd_);
  }

  template <typename Callback>
  void run(Callback&& callback) {
    std::uint64_t captured_buffers = 0;
    std::uint64_t invalid_jpegs = 0;
    const auto capture_started = std::chrono::steady_clock::now();
    while (g_running.load()) {
      pollfd descriptor{camera_fd_, POLLIN, 0};
      const int ready = poll(&descriptor, 1, 2500);
      if (ready == 0) throw std::runtime_error("camera frame timeout");
      if (ready < 0) {
        if (errno == EINTR) continue;
        throw std::runtime_error("camera poll failed: " + std::string(strerror(errno)));
      }
      if (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) {
        throw std::runtime_error("camera disconnected");
      }

      v4l2_buffer buffer{};
      buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buffer.memory = V4L2_MEMORY_MMAP;
      if (xioctl(camera_fd_, VIDIOC_DQBUF, &buffer) < 0) {
        if (errno == EAGAIN) continue;
        throw std::runtime_error("VIDIOC_DQBUF failed: " + std::string(strerror(errno)));
      }
      if (buffer.index >= buffers_.size() || buffer.bytesused > buffers_[buffer.index].length) {
        throw std::runtime_error("invalid V4L2 buffer returned by driver");
      }

      const auto* data = static_cast<const std::uint8_t*>(buffers_[buffer.index].address);
      ++captured_buffers;
      std::size_t jpeg_size = buffer.bytesused;
      if (jpeg_size >= 4 && data[0] == 0xFF && data[1] == 0xD8) {
        while (jpeg_size >= 4 && !(data[jpeg_size - 2] == 0xFF && data[jpeg_size - 1] == 0xD9)) {
          --jpeg_size;
        }
        if (jpeg_size >= 4) {
          callback(data, jpeg_size, monotonic_nanoseconds());
        } else {
          ++invalid_jpegs;
        }
      } else {
        ++invalid_jpegs;
      }
      if (captured_buffers % 50 == 0) {
        const double seconds = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - capture_started).count();
        std::cout << "captured=" << captured_buffers << " capture_fps="
                  << captured_buffers / seconds << " invalid_jpegs=" << invalid_jpegs << std::endl;
      }

      if (xioctl(camera_fd_, VIDIOC_QBUF, &buffer) < 0) {
        throw std::runtime_error("VIDIOC_QBUF failed after capture");
      }
    }
  }

 private:
  Options options_;
  int camera_fd_ = -1;
  bool streaming_ = false;
  std::vector<MappedBuffer> buffers_;
};

class UdpSender {
 public:
  explicit UdpSender(const Options& options) : options_(options) {
    socket_fd_ = socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
    if (socket_fd_ < 0) throw std::runtime_error("UDP socket creation failed");
    int send_buffer = 1024 * 1024;
    setsockopt(socket_fd_, SOL_SOCKET, SO_SNDBUF, &send_buffer, sizeof(send_buffer));
    int low_delay = IPTOS_LOWDELAY;
    setsockopt(socket_fd_, IPPROTO_IP, IP_TOS, &low_delay, sizeof(low_delay));

    peer_.sin_family = AF_INET;
    peer_.sin_port = htons(options.port);
    if (inet_pton(AF_INET, options.peer_ip.c_str(), &peer_.sin_addr) != 1) {
      throw std::runtime_error("invalid peer IP address");
    }
    started_ = std::chrono::steady_clock::now();
  }

  ~UdpSender() {
    if (socket_fd_ >= 0) close(socket_fd_);
  }

  void send_frame(const std::uint8_t* data, std::size_t size, std::uint64_t capture_ns) {
    const std::size_t count = (size + kPayloadBytes - 1) / kPayloadBytes;
    if (count == 0 || count > UINT16_MAX || size > UINT32_MAX) return;
    const std::uint32_t checksum = crc32(data, size);
    const std::uint32_t current_frame = frame_id_++;
    std::vector<std::uint8_t> datagram(sizeof(WireHeader) + kPayloadBytes);

    for (std::size_t index = 0; index < count; ++index) {
      const std::size_t offset = index * kPayloadBytes;
      const std::size_t payload_size = std::min(kPayloadBytes, size - offset);
      WireHeader header{};
      std::memcpy(header.magic, kMagic, sizeof(kMagic));
      header.version = kVersion;
      header.header_bytes = htons(sizeof(WireHeader));
      header.frame_id = htonl(current_frame);
      header.capture_monotonic_ns = host_to_network_64(capture_ns);
      header.frame_size = htonl(static_cast<std::uint32_t>(size));
      header.crc32 = htonl(checksum);
      header.width = htons(options_.width);
      header.height = htons(options_.height);
      header.fragment_index = htons(static_cast<std::uint16_t>(index));
      header.fragment_count = htons(static_cast<std::uint16_t>(count));
      header.payload_size = htons(static_cast<std::uint16_t>(payload_size));
      std::memcpy(datagram.data(), &header, sizeof(header));
      std::memcpy(datagram.data() + sizeof(header), data + offset, payload_size);
      const ssize_t sent = sendto(socket_fd_, datagram.data(), sizeof(header) + payload_size, 0,
                                  reinterpret_cast<const sockaddr*>(&peer_), sizeof(peer_));
      if (sent < 0) {
        ++send_errors_;
        return;
      }
    }

    ++frames_sent_;
    bytes_sent_ += size;
    if (frames_sent_ % 50 == 0) {
      const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - started_).count();
      std::cout << "frames=" << frames_sent_ << " fps=" << frames_sent_ / seconds
                << " jpeg_kib=" << size / 1024.0 << " send_errors=" << send_errors_ << std::endl;
    }
  }

 private:
  Options options_;
  int socket_fd_ = -1;
  sockaddr_in peer_{};
  std::uint32_t frame_id_ = 0;
  std::uint64_t frames_sent_ = 0;
  std::uint64_t bytes_sent_ = 0;
  std::uint64_t send_errors_ = 0;
  std::chrono::steady_clock::time_point started_;
};

}  // namespace

int main(int argc, char** argv) {
  std::signal(SIGINT, handle_signal);
  std::signal(SIGTERM, handle_signal);
  try {
    const Options options = parse_options(argc, argv);
    UdpSender sender(options);
    while (g_running.load()) {
      try {
        std::cout << "opening " << options.device << " as MJPEG " << options.width << 'x'
                  << options.height << '@' << options.fps << " -> " << options.peer_ip << ':'
                  << options.port << std::endl;
        CameraStream camera(options);
        camera.run([&](const std::uint8_t* data, std::size_t size, std::uint64_t capture_ns) {
          sender.send_frame(data, size, capture_ns);
        });
      } catch (const std::exception& error) {
        if (!g_running.load()) break;
        std::cerr << "camera stream error: " << error.what() << "; retrying in 2 seconds" << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(2));
      }
    }
  } catch (const std::exception& error) {
    std::cerr << "fatal: " << error.what() << std::endl;
    return 1;
  }
  return 0;
}
