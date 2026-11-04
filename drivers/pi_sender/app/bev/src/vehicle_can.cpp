#include "w102/vehicle_can.hpp"

#include "w102/udp_protocol.hpp"

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <utility>

namespace w102 {
namespace {

constexpr std::array<std::uint32_t, 7> kFeedbackIds{
    0x352, 0x354, 0x356, 0x358, 0x360, 0x362, 0x364};

std::uint64_t packed_le(const std::uint8_t* data, std::size_t size) {
  std::uint64_t packed = 0;
  for (std::size_t index = 0; index < std::min<std::size_t>(size, 8); ++index) {
    packed |= static_cast<std::uint64_t>(data[index]) << (index * 8U);
  }
  return packed;
}

std::uint64_t unsigned_signal(std::uint64_t packed, unsigned start_bit,
                              unsigned bit_length) {
  if (bit_length == 0 || bit_length > 63) return 0;
  return (packed >> start_bit) & ((std::uint64_t{1} << bit_length) - 1U);
}

std::int64_t signed_signal(std::uint64_t packed, unsigned start_bit,
                           unsigned bit_length) {
  const auto raw = unsigned_signal(packed, start_bit, bit_length);
  const auto sign = std::uint64_t{1} << (bit_length - 1U);
  if ((raw & sign) == 0) return static_cast<std::int64_t>(raw);
  return static_cast<std::int64_t>(raw | (~std::uint64_t{0} << bit_length));
}

double age_ms(std::uint64_t timestamp, std::uint64_t now_ns) {
  if (timestamp == 0) return -1.0;
  // The reader and health loop are concurrent. A frame can arrive just after
  // now_ns was sampled, so clamp that sub-cycle race to zero age.
  if (now_ns < timestamp) return 0.0;
  return static_cast<double>(now_ns - timestamp) / 1000000.0;
}

bool fresh(std::uint64_t timestamp, std::uint64_t now_ns, int stale_ms) {
  const double age = age_ms(timestamp, now_ns);
  return age >= 0.0 && age <= stale_ms;
}

const char* control_mode_name(int value) {
  switch (value) {
    case 1: return "automatic";
    case 2: return "remote";
    default: return "invalid_or_other";
  }
}

const char* stop_state_name(int value) {
  switch (value) {
    case 0: return "none";
    case 1: return "collision_stop";
    case 2: return "mechanical_emergency_stop";
    case 3: return "can_emergency_stop";
    case 4: return "fault_emergency_stop";
    default: return "unknown";
  }
}

const char* gear_name(int value) {
  switch (value) {
    case 0: return "N";
    case 1: return "D";
    case 2: return "R";
    default: return "UNKNOWN";
  }
}

const char* turn_signal_name(int value) {
  switch (value) {
    case 0: return "off";
    case 1: return "left";
    case 2: return "right";
    default: return "unknown";
  }
}

void append_number_or_null(std::ostringstream& output, bool valid,
                           double value) {
  if (valid && std::isfinite(value)) {
    output << value;
  } else {
    output << "null";
  }
}

struct OpenCanSocket {
  std::string interface;
  int fd{-1};
};

int open_can_socket(const std::string& interface) {
  const int fd = socket(PF_CAN, SOCK_RAW | SOCK_CLOEXEC, CAN_RAW);
  if (fd < 0) return -1;

  std::array<can_filter, kFeedbackIds.size()> filters{};
  for (std::size_t index = 0; index < filters.size(); ++index) {
    filters[index].can_id = kFeedbackIds[index];
    filters[index].can_mask = CAN_SFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG;
  }
  if (setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FILTER, filters.data(),
                 sizeof(filters)) < 0) {
    close(fd);
    return -1;
  }
  const int receive_own = 0;
  setsockopt(fd, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &receive_own,
             sizeof(receive_own));

  ifreq request{};
  const auto length = std::min(interface.size(), sizeof(request.ifr_name) - 1U);
  std::copy_n(interface.data(), length, request.ifr_name);
  if (ioctl(fd, SIOCGIFINDEX, &request) < 0) {
    close(fd);
    return -1;
  }
  sockaddr_can address{};
  address.can_family = AF_CAN;
  address.can_ifindex = request.ifr_ifindex;
  if (bind(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0) {
    close(fd);
    return -1;
  }
  const int flags = fcntl(fd, F_GETFL, 0);
  if (flags >= 0) fcntl(fd, F_SETFL, flags | O_NONBLOCK);
  return fd;
}

}  // namespace

bool decode_vehicle_can_frame(VehicleCanStatus& status,
                              std::uint32_t can_id,
                              const std::uint8_t* data,
                              std::size_t size,
                              std::uint64_t received_ns,
                              const std::string& source_interface) {
  if (data == nullptr || size < 8) return false;
  const auto packed = packed_le(data, size);
  std::size_t message_index = 0;
  switch (can_id) {
    case 0x352: {
      message_index = 0;
      status.control_mode = static_cast<int>(unsigned_signal(packed, 0, 2));
      status.stop_state = static_cast<int>(unsigned_signal(packed, 2, 3));
      status.ignition_on = unsigned_signal(packed, 5, 1) != 0;
      status.vcu_fault = static_cast<int>(unsigned_signal(packed, 8, 3));
      status.vcu_software_raw =
          static_cast<std::uint32_t>(unsigned_signal(packed, 16, 28));
      std::uint8_t checksum = 0;
      for (std::size_t index = 0; index < 7; ++index) checksum ^= data[index];
      status.vcu_checksum_ok = checksum == data[7];
      break;
    }
    case 0x354:
      message_index = 1;
      status.gear = static_cast<int>(unsigned_signal(packed, 0, 2));
      status.speed_mps = signed_signal(packed, 2, 12) * 0.01;
      status.acceleration_mps2 = signed_signal(packed, 14, 12) * 0.01;
      status.motor_torque = static_cast<int>(unsigned_signal(packed, 26, 12));
      status.motor_rpm = static_cast<int>(unsigned_signal(packed, 38, 16));
      status.motor_error_level =
          static_cast<int>(unsigned_signal(packed, 54, 2));
      status.motor_fault = static_cast<int>(unsigned_signal(packed, 56, 8));
      break;
    case 0x356:
      message_index = 2;
      status.epb_state = static_cast<int>(unsigned_signal(packed, 0, 3));
      status.epb_error_level =
          static_cast<int>(unsigned_signal(packed, 8, 8));
      status.epb_fault = static_cast<int>(unsigned_signal(packed, 16, 8));
      break;
    case 0x358:
      message_index = 3;
      status.front_wheel_angle = signed_signal(packed, 0, 16) * 0.1;
      status.eps_work_state = static_cast<int>(unsigned_signal(packed, 16, 4));
      status.eps_error_level =
          static_cast<int>(unsigned_signal(packed, 20, 4));
      status.eps_fault = static_cast<int>(unsigned_signal(packed, 24, 8));
      break;
    case 0x360:
      message_index = 4;
      status.brake_deceleration_mps2 = signed_signal(packed, 0, 12) * 0.01;
      status.brake_pressure = static_cast<int>(unsigned_signal(packed, 16, 8));
      status.brake_error_level =
          static_cast<int>(unsigned_signal(packed, 24, 8));
      status.brake_fault = static_cast<int>(unsigned_signal(packed, 32, 8));
      break;
    case 0x362: {
      message_index = 5;
      status.charging = unsigned_signal(packed, 0, 1) != 0;
      const auto soc_raw = unsigned_signal(packed, 1, 12);
      status.soc_percent = soc_raw * 0.1;
      status.soc_valid = soc_raw <= 1000;
      const auto current_raw = unsigned_signal(packed, 16, 16);
      status.battery_current_a = signed_signal(packed, 16, 16) * 0.1;
      status.battery_current_valid = current_raw != 0x3FFF;
      const auto voltage_raw = unsigned_signal(packed, 32, 16);
      status.battery_voltage_v = voltage_raw * 0.1;
      status.battery_voltage_valid = voltage_raw != 0x1FFF;
      status.bms_error_level =
          static_cast<int>(unsigned_signal(packed, 13, 3));
      status.bms_fault = static_cast<int>(unsigned_signal(packed, 48, 8));
      break;
    }
    case 0x364:
      message_index = 6;
      status.headlight = static_cast<int>(unsigned_signal(packed, 0, 2));
      status.turn_signal = static_cast<int>(unsigned_signal(packed, 2, 2));
      status.reverse_light = static_cast<int>(unsigned_signal(packed, 4, 2));
      status.brake_light = static_cast<int>(unsigned_signal(packed, 6, 2));
      status.horn = static_cast<int>(unsigned_signal(packed, 8, 2));
      status.external_voice = static_cast<int>(unsigned_signal(packed, 10, 2));
      status.hazard_light = static_cast<int>(unsigned_signal(packed, 12, 2));
      break;
    default:
      return false;
  }
  status.source_interface = source_interface;
  ++status.received_frames;
  status.last_frame_ns = received_ns;
  status.message_mask |= std::uint32_t{1} << message_index;
  status.message_times_ns[message_index] = received_ns;
  return true;
}

std::string vehicle_can_status_json(const VehicleCanStatus& status,
                                    std::uint64_t now_ns,
                                    int stale_ms) {
  const bool can_online = fresh(status.last_frame_ns, now_ns, stale_ms);
  const bool vcu_fresh = fresh(status.message_times_ns[0], now_ns, stale_ms);
  const bool motor_fresh = fresh(status.message_times_ns[1], now_ns, stale_ms);
  const bool epb_fresh = fresh(status.message_times_ns[2], now_ns, stale_ms);
  const bool eps_fresh = fresh(status.message_times_ns[3], now_ns, stale_ms);
  const bool brake_fresh = fresh(status.message_times_ns[4], now_ns, stale_ms);
  const bool bms_fresh = fresh(status.message_times_ns[5], now_ns, stale_ms);
  const bool body_fresh = fresh(status.message_times_ns[6], now_ns, stale_ms);
  const bool braking =
      (brake_fresh && status.brake_pressure > 0) ||
      (body_fresh && status.brake_light == 1) ||
      (epb_fresh && (status.epb_state == 2 || status.epb_state == 3));
  const bool drive_output = motor_fresh && status.gear != 0 &&
                            (status.motor_torque > 0 ||
                             std::abs(status.speed_mps) >= 0.02);

  std::ostringstream output;
  output << std::fixed << std::setprecision(3);
  output << "{\"can\":{\"online\":" << (can_online ? "true" : "false")
         << ",\"read_only\":true,\"interface\":\""
         << status.source_interface << "\",\"bitrate\":500000"
         << ",\"received_frames\":" << status.received_frames
         << ",\"message_mask\":" << status.message_mask
         << ",\"last_frame_age_ms\":";
  append_number_or_null(output, status.last_frame_ns != 0,
                        age_ms(status.last_frame_ns, now_ns));
  output << "},\"vehicle\":{"
         << "\"control\":{\"fresh\":" << (vcu_fresh ? "true" : "false")
         << ",\"age_ms\":";
  append_number_or_null(output, status.message_times_ns[0] != 0,
                        age_ms(status.message_times_ns[0], now_ns));
  output << ",\"mode_raw\":" << status.control_mode
         << ",\"mode\":\"" << control_mode_name(status.control_mode) << "\""
         << ",\"stop_state_raw\":" << status.stop_state
         << ",\"stop_state\":\"" << stop_state_name(status.stop_state) << "\""
         << ",\"ignition_on\":" << (status.ignition_on ? "true" : "false")
         << ",\"vcu_fault\":" << status.vcu_fault
         << ",\"vcu_software_raw\":" << status.vcu_software_raw
         << ",\"checksum_ok\":" << (status.vcu_checksum_ok ? "true" : "false")
         << "},\"motion\":{\"fresh\":" << (motor_fresh ? "true" : "false")
         << ",\"age_ms\":";
  append_number_or_null(output, status.message_times_ns[1] != 0,
                        age_ms(status.message_times_ns[1], now_ns));
  output << ",\"gear_raw\":" << status.gear << ",\"gear\":\""
         << gear_name(status.gear) << "\",\"speed_mps\":" << status.speed_mps
         << ",\"speed_kph\":" << status.speed_mps * 3.6
         << ",\"acceleration_mps2\":" << status.acceleration_mps2
         << ",\"motor_torque\":" << status.motor_torque
         << ",\"motor_rpm\":" << status.motor_rpm
         << ",\"drive_output_active\":" << (drive_output ? "true" : "false")
         << ",\"throttle_signal_available\":false"
         << ",\"motor_error_level\":" << status.motor_error_level
         << ",\"motor_fault\":" << status.motor_fault
         << "},\"steering\":{\"fresh\":" << (eps_fresh ? "true" : "false")
         << ",\"age_ms\":";
  append_number_or_null(output, status.message_times_ns[3] != 0,
                        age_ms(status.message_times_ns[3], now_ns));
  output << ",\"front_wheel_angle\":" << status.front_wheel_angle
         << ",\"work_state\":" << status.eps_work_state
         << ",\"error_level\":" << status.eps_error_level
         << ",\"fault\":" << status.eps_fault
         << "},\"brake\":{\"fresh\":" << (brake_fresh ? "true" : "false")
         << ",\"age_ms\":";
  append_number_or_null(output, status.message_times_ns[4] != 0,
                        age_ms(status.message_times_ns[4], now_ns));
  output << ",\"active\":" << (braking ? "true" : "false")
         << ",\"deceleration_mps2\":" << status.brake_deceleration_mps2
         << ",\"pressure\":" << status.brake_pressure
         << ",\"error_level\":" << status.brake_error_level
         << ",\"fault\":" << status.brake_fault
         << "},\"parking_brake\":{\"fresh\":" << (epb_fresh ? "true" : "false")
         << ",\"age_ms\":";
  append_number_or_null(output, status.message_times_ns[2] != 0,
                        age_ms(status.message_times_ns[2], now_ns));
  output << ",\"state\":" << status.epb_state
         << ",\"error_level\":" << status.epb_error_level
         << ",\"fault\":" << status.epb_fault
         << "},\"battery\":{\"fresh\":" << (bms_fresh ? "true" : "false")
         << ",\"age_ms\":";
  append_number_or_null(output, status.message_times_ns[5] != 0,
                        age_ms(status.message_times_ns[5], now_ns));
  output << ",\"charging\":" << (status.charging ? "true" : "false")
         << ",\"soc_percent\":";
  append_number_or_null(output, status.soc_valid, status.soc_percent);
  output << ",\"current_a\":";
  append_number_or_null(output, status.battery_current_valid,
                        status.battery_current_a);
  output << ",\"voltage_v\":";
  append_number_or_null(output, status.battery_voltage_valid,
                        status.battery_voltage_v);
  output << ",\"error_level\":" << status.bms_error_level
         << ",\"fault\":" << status.bms_fault
         << "},\"lights\":{\"fresh\":" << (body_fresh ? "true" : "false")
         << ",\"age_ms\":";
  append_number_or_null(output, status.message_times_ns[6] != 0,
                        age_ms(status.message_times_ns[6], now_ns));
  output << ",\"headlight\":" << status.headlight
         << ",\"turn_signal_raw\":" << status.turn_signal
         << ",\"turn_signal\":\"" << turn_signal_name(status.turn_signal) << "\""
         << ",\"reverse_light\":" << status.reverse_light
         << ",\"brake_light\":" << status.brake_light
         << ",\"horn\":" << status.horn
         << ",\"external_voice\":" << status.external_voice
         << ",\"hazard_light\":" << status.hazard_light
         << "}}}";
  return output.str();
}

VehicleCanReader::VehicleCanReader(std::vector<std::string> interfaces,
                                   int stale_ms)
    : interfaces_(std::move(interfaces)), stale_ms_(stale_ms) {}

VehicleCanReader::~VehicleCanReader() { stop(); }

void VehicleCanReader::start() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (thread_.joinable()) return;
  stop_requested_ = false;
  thread_ = std::thread(&VehicleCanReader::run, this);
}

void VehicleCanReader::stop() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stop_requested_ = true;
  }
  if (thread_.joinable()) thread_.join();
}

VehicleCanStatus VehicleCanReader::snapshot() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return status_;
}

void VehicleCanReader::run() {
  std::vector<OpenCanSocket> sockets;
  auto next_open = std::chrono::steady_clock::now();
  while (true) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (stop_requested_) break;
    }
    const auto now = std::chrono::steady_clock::now();
    if (now >= next_open) {
      for (const auto& interface : interfaces_) {
        const auto found = std::find_if(
            sockets.begin(), sockets.end(), [&](const OpenCanSocket& entry) {
              return entry.interface == interface;
            });
        if (found != sockets.end()) continue;
        const int fd = open_can_socket(interface);
        if (fd >= 0) sockets.push_back({interface, fd});
      }
      next_open = now + std::chrono::seconds(2);
    }

    std::vector<pollfd> descriptors;
    descriptors.reserve(sockets.size());
    for (const auto& entry : sockets) {
      descriptors.push_back({entry.fd, POLLIN, 0});
    }
    const int result = descriptors.empty()
                           ? (std::this_thread::sleep_for(
                                  std::chrono::milliseconds(250)),
                              0)
                           : poll(descriptors.data(), descriptors.size(), 250);
    if (result <= 0) continue;
    for (std::size_t index = 0; index < descriptors.size(); ++index) {
      if ((descriptors[index].revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
        close(sockets[index].fd);
        sockets[index].fd = -1;
        continue;
      }
      if ((descriptors[index].revents & POLLIN) == 0) continue;
      can_frame frame{};
      while (read(sockets[index].fd, &frame, sizeof(frame)) == sizeof(frame)) {
        const auto can_id = frame.can_id & CAN_SFF_MASK;
        const auto received_ns = monotonic_time_ns();
        std::lock_guard<std::mutex> lock(mutex_);
        decode_vehicle_can_frame(status_, can_id, frame.data, frame.can_dlc,
                                 received_ns, sockets[index].interface);
      }
    }
    sockets.erase(
        std::remove_if(sockets.begin(), sockets.end(),
                       [](const OpenCanSocket& entry) { return entry.fd < 0; }),
        sockets.end());
  }
  for (const auto& entry : sockets) close(entry.fd);
}

}  // namespace w102
