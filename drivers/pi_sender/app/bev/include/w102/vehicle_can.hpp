#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace w102 {

struct VehicleCanStatus {
  std::string source_interface;
  std::uint64_t received_frames{0};
  std::uint64_t last_frame_ns{0};
  std::uint32_t message_mask{0};
  std::array<std::uint64_t, 7> message_times_ns{};

  int control_mode{0};
  int stop_state{0};
  bool ignition_on{false};
  int vcu_fault{0};
  std::uint32_t vcu_software_raw{0};
  bool vcu_checksum_ok{false};

  int gear{0};
  double speed_mps{0.0};
  double acceleration_mps2{0.0};
  int motor_torque{0};
  int motor_rpm{0};
  int motor_error_level{0};
  int motor_fault{0};

  int epb_state{0};
  int epb_error_level{0};
  int epb_fault{0};

  double front_wheel_angle{0.0};
  int eps_work_state{0};
  int eps_error_level{0};
  int eps_fault{0};

  double brake_deceleration_mps2{0.0};
  int brake_pressure{0};
  int brake_error_level{0};
  int brake_fault{0};

  bool charging{false};
  double soc_percent{0.0};
  bool soc_valid{false};
  double battery_current_a{0.0};
  bool battery_current_valid{false};
  double battery_voltage_v{0.0};
  bool battery_voltage_valid{false};
  int bms_error_level{0};
  int bms_fault{0};

  int headlight{0};
  int turn_signal{0};
  int reverse_light{0};
  int brake_light{0};
  int horn{0};
  int external_voice{0};
  int hazard_light{0};
};

// Decodes one W102 feedback frame. This function never transmits to the CAN
// bus and returns false for identifiers outside the feedback whitelist.
bool decode_vehicle_can_frame(VehicleCanStatus& status,
                              std::uint32_t can_id,
                              const std::uint8_t* data,
                              std::size_t size,
                              std::uint64_t received_ns,
                              const std::string& source_interface);

std::string vehicle_can_status_json(const VehicleCanStatus& status,
                                    std::uint64_t now_ns,
                                    int stale_ms);

class VehicleCanReader {
 public:
  explicit VehicleCanReader(std::vector<std::string> interfaces,
                            int stale_ms = 500);
  ~VehicleCanReader();
  VehicleCanReader(const VehicleCanReader&) = delete;
  VehicleCanReader& operator=(const VehicleCanReader&) = delete;

  void start();
  void stop();
  VehicleCanStatus snapshot() const;
  int stale_ms() const { return stale_ms_; }

 private:
  void run();

  std::vector<std::string> interfaces_;
  int stale_ms_{500};
  mutable std::mutex mutex_;
  VehicleCanStatus status_;
  bool stop_requested_{false};
  std::thread thread_;
};

}  // namespace w102
