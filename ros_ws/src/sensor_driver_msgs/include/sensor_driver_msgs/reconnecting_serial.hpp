#ifndef SENSOR_DRIVER_MSGS__RECONNECTING_SERIAL_HPP_
#define SENSOR_DRIVER_MSGS__RECONNECTING_SERIAL_HPP_

#include <serial/serial.h>

#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <functional>
#include <iomanip>
#include <sstream>
#include <cstring>
#include <stdexcept>
#include <string>
#include <utility>

namespace sensor_driver_msgs
{

class StableDeviceLock
{
public:
  explicit StableDeviceLock(const std::string & stable_device_path)
  {
    // Hash the configured stable path itself.  Resolving a symlink here would make
    // the lock name change between the connected and disconnected states.
    uint64_t hash = 14695981039346656037ULL;
    for (const unsigned char value : stable_device_path) {
      hash ^= value;
      hash *= 1099511628211ULL;
    }
    const std::string runtime_dir = "/run/user/" + std::to_string(getuid());
    const std::string lock_dir = access(runtime_dir.c_str(), W_OK) == 0 ? runtime_dir : "/tmp";
    std::ostringstream name;
    name << lock_dir << "/white_ws_" << getuid() << "_serial_" << std::hex <<
      std::setw(16) << std::setfill('0') << hash << ".lock";
    path_ = name.str();

    fd_ = open(path_.c_str(), O_CREAT | O_CLOEXEC | O_RDWR | O_NOFOLLOW, 0600);
    if (fd_ < 0 || flock(fd_, LOCK_EX | LOCK_NB) != 0) {
      if (fd_ >= 0) {
        close(fd_);
        fd_ = -1;
      }
      const int error_number = errno;
      throw std::runtime_error(
              "cannot lock serial device " + stable_device_path + " at " + path_ + ": " +
              std::strerror(error_number));
    }
  }

  ~StableDeviceLock()
  {
    if (fd_ >= 0) {
      close(fd_);
    }
  }

  StableDeviceLock(const StableDeviceLock &) = delete;
  StableDeviceLock & operator=(const StableDeviceLock &) = delete;

private:
  int fd_{-1};
  std::string path_;
};

// Owns one receive-only serial connection and its health state.  A port is not
// considered healthy merely because it can be opened: the protocol parser must
// call mark_valid_frame() whenever it accepts a checksum-valid frame.
class ReconnectingSerial
{
public:
  using Clock = std::chrono::steady_clock;
  using LogCallback = std::function<void(const std::string &)>;

  struct Options
  {
    std::string device;
    uint32_t baudrate{115200};
    std::chrono::milliseconds read_timeout{20};
    std::chrono::milliseconds reconnect_period{1000};
    std::chrono::milliseconds data_timeout{2000};
  };

  ReconnectingSerial(
    Options options, LogCallback error_callback, LogCallback recovered_callback)
  : options_(std::move(options)),
    device_lock_(options_.device),
    error_callback_(std::move(error_callback)),
    recovered_callback_(std::move(recovered_callback)),
    last_open_attempt_(Clock::now() - options_.reconnect_period),
    last_error_report_(Clock::now())
  {
    if (options_.device.empty()) {
      throw std::invalid_argument("serial device path cannot be empty");
    }
    if (options_.reconnect_period < std::chrono::milliseconds(100) ||
      options_.data_timeout < std::chrono::milliseconds(100))
    {
      throw std::invalid_argument("serial reconnect and data timeouts must be at least 100 ms");
    }
    serial_.setPort(options_.device);
    serial_.setBaudrate(options_.baudrate);
    auto timeout = serial::Timeout::simpleTimeout(
      static_cast<uint32_t>(options_.read_timeout.count()));
    serial_.setTimeout(timeout);
    set_offline_reason("device is not connected");
  }

  ~ReconnectingSerial()
  {
    close_noexcept();
  }

  ReconnectingSerial(const ReconnectingSerial &) = delete;
  ReconnectingSerial & operator=(const ReconnectingSerial &) = delete;

  bool service()
  {
    const auto current = Clock::now();
    if (serial_.isOpen()) {
      if (current - last_valid_frame_ >= options_.data_timeout) {
        set_offline_reason(
          "no valid protocol frame for " + std::to_string(options_.data_timeout.count()) +
          " ms; closing port and reconnecting");
        close_noexcept();
        last_open_attempt_ = current;
        report_offline_if_due(current);
        return false;
      } else {
        report_offline_if_due(current);
        return true;
      }
    }

    if (current - last_open_attempt_ < options_.reconnect_period) {
      report_offline_if_due(current);
      return false;
    }
    last_open_attempt_ = current;
    try {
      serial_.open();
      reset_pending_ = true;
      last_valid_frame_ = current;
      set_offline_reason("port opened; waiting for a valid protocol frame");
      return true;
    } catch (const std::exception & error) {
      set_offline_reason(std::string("cannot open serial port: ") + error.what());
      close_noexcept();
      report_offline_if_due(current);
      return false;
    }
  }

  std::size_t read(uint8_t * destination, const std::size_t capacity)
  {
    if (!serial_.isOpen() || capacity == 0) {
      return 0;
    }
    try {
      const std::size_t available = serial_.available();
      if (available == 0) {
        return 0;
      }
      return serial_.read(destination, std::min(available, capacity));
    } catch (const std::exception & error) {
      set_offline_reason(std::string("serial read error: ") + error.what());
      const auto current = Clock::now();
      last_open_attempt_ = current;
      close_noexcept();
      report_offline_if_due(current);
      return 0;
    }
  }

  void mark_valid_frame()
  {
    last_valid_frame_ = Clock::now();
    if (offline_) {
      offline_ = false;
      if (recovered_callback_) {
        recovered_callback_("valid serial data restored on " + options_.device);
      }
    }
  }

  bool is_open() const
  {
    return serial_.isOpen();
  }

  bool take_connection_reset()
  {
    return std::exchange(reset_pending_, false);
  }

  const std::string & device() const
  {
    return options_.device;
  }

private:
  static constexpr std::chrono::seconds kErrorReportPeriod{5};

  void set_offline_reason(const std::string & reason)
  {
    offline_ = true;
    offline_reason_ = reason;
  }

  void report_offline_if_due(const Clock::time_point current)
  {
    if (!offline_ || current - last_error_report_ < kErrorReportPeriod) {
      return;
    }
    last_error_report_ = current;
    if (error_callback_) {
      error_callback_(options_.device + ": " + offline_reason_);
    }
  }

  void close_noexcept()
  {
    if (serial_.isOpen()) {
      reset_pending_ = true;
    }
    try {
      if (serial_.isOpen()) {
        serial_.close();
      }
    } catch (const std::exception &) {
    }
  }

  Options options_;
  StableDeviceLock device_lock_;
  serial::Serial serial_;
  LogCallback error_callback_;
  LogCallback recovered_callback_;
  Clock::time_point last_open_attempt_;
  Clock::time_point last_valid_frame_{};
  Clock::time_point last_error_report_;
  bool offline_{true};
  bool reset_pending_{false};
  std::string offline_reason_;
};

}  // namespace sensor_driver_msgs

#endif  // SENSOR_DRIVER_MSGS__RECONNECTING_SERIAL_HPP_
