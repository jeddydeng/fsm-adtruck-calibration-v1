#include "w102/adaptive_illumination.hpp"
#include "w102/bev_composer.hpp"
#include "w102/udp_protocol.hpp"
#include "w102/vehicle_can.hpp"

#include <arpa/inet.h>
#include <poll.h>
#include <sys/resource.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cctype>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <future>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

namespace {

std::atomic<bool> g_running{true};
void handle_signal(int) { g_running.store(false); }

struct Options {
  std::vector<std::string> devices;
  std::vector<std::string> calibration_files;
  std::string peer_ip{"192.168.1.102"};
  std::uint16_t data_port{5600};
  std::uint16_t control_port{5601};
  int width{1280};
  int height{720};
  int fps{10};
  int jpeg_quality{80};
  int raw_stream_start{0};
  bool send_raw{false};
  std::string default_output_mode{"bev"};
  int bev_width{1024};
  int bev_height{1024};
  int camera_stale_ms{300};
  bool adaptive_illumination{true};
  int thermal_degrade_c{75};
  int thermal_pause_c{82};
  bool draw_vehicle{true};
  double vehicle_length_m{2.0};
  double vehicle_width_m{1.2};
  double meters_per_pixel{0.0125};
  std::string pressure_log_dir;
  std::string pressure_test_state;
  std::vector<std::string> can_interfaces;
  int can_stale_ms{500};
};

struct RuntimeSettings {
  std::atomic<bool> send_raw{false};
  std::atomic<bool> send_bev{true};
  std::atomic<std::uint16_t> raw_stream_mask{0};
  std::atomic<int> output_fps{10};
  std::atomic<int> jpeg_quality{80};
};

constexpr std::uint16_t kBevOutputBit = 1U << 15U;

int parse_int(const std::string& name, const char* value, int minimum,
              int maximum) {
  const long parsed = std::stol(value);
  if (parsed < minimum || parsed > maximum) {
    throw std::invalid_argument(name + " out of range");
  }
  return static_cast<int>(parsed);
}

double parse_double(const std::string& name, const char* value,
                    double minimum, double maximum) {
  const double parsed = std::stod(value);
  if (!std::isfinite(parsed) || parsed < minimum || parsed > maximum) {
    throw std::invalid_argument(name + " out of range");
  }
  return parsed;
}

Options parse_options(int argc, char** argv) {
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    auto next = [&]() -> const char* {
      if (++index >= argc) {
        throw std::invalid_argument("missing value for " + argument);
      }
      return argv[index];
    };
    if (argument == "--device" || argument == "--camera") {
      options.devices.emplace_back(next());
    } else if (argument == "--peer-ip") {
      options.peer_ip = next();
    } else if (argument == "--data-port") {
      options.data_port = parse_int(argument, next(), 1, 65535);
    } else if (argument == "--control-port") {
      options.control_port = parse_int(argument, next(), 1, 65535);
    } else if (argument == "--width") {
      options.width = parse_int(argument, next(), 160, 8192);
    } else if (argument == "--height") {
      options.height = parse_int(argument, next(), 120, 8192);
    } else if (argument == "--fps") {
      options.fps = parse_int(argument, next(), 10, 30);
    } else if (argument == "--jpeg-quality") {
      options.jpeg_quality = parse_int(argument, next(), 20, 100);
    } else if (argument == "--stream-id") {
      options.raw_stream_start = parse_int(argument, next(), 0, 11);
    } else if (argument == "--send-raw") {
      options.send_raw = parse_int(argument, next(), 0, 1) != 0;
    } else if (argument == "--default-output-mode") {
      options.default_output_mode = next();
    } else if (argument == "--calibration") {
      options.calibration_files.emplace_back(next());
    } else if (argument == "--bev-width") {
      options.bev_width = parse_int(argument, next(), 128, 8192);
    } else if (argument == "--bev-height") {
      options.bev_height = parse_int(argument, next(), 128, 8192);
    } else if (argument == "--camera-stale-ms") {
      options.camera_stale_ms = parse_int(argument, next(), 100, 5000);
    } else if (argument == "--adaptive-illumination") {
      options.adaptive_illumination = parse_int(argument, next(), 0, 1) != 0;
    } else if (argument == "--thermal-degrade-c") {
      options.thermal_degrade_c = parse_int(argument, next(), 55, 85);
    } else if (argument == "--thermal-pause-c") {
      options.thermal_pause_c = parse_int(argument, next(), 65, 90);
    } else if (argument == "--draw-vehicle") {
      options.draw_vehicle = parse_int(argument, next(), 0, 1) != 0;
    } else if (argument == "--vehicle-length-m") {
      options.vehicle_length_m = parse_double(argument, next(), 0.1, 20.0);
    } else if (argument == "--vehicle-width-m") {
      options.vehicle_width_m = parse_double(argument, next(), 0.1, 10.0);
    } else if (argument == "--meters-per-pixel") {
      options.meters_per_pixel = parse_double(argument, next(), 0.001, 0.2);
    } else if (argument == "--pressure-log-dir") {
      options.pressure_log_dir = next();
    } else if (argument == "--pressure-test-state") {
      options.pressure_test_state = next();
    } else if (argument == "--can-interface") {
      options.can_interfaces.emplace_back(next());
    } else if (argument == "--can-stale-ms") {
      options.can_stale_ms = parse_int(argument, next(), 50, 10000);
    } else if (argument == "--help") {
      std::cout
          << "w102_camera_gateway options:\n"
          << "  --camera PATH (repeat 1-4 times; --device is an alias)\n"
          << "  --calibration YAML (repeat once per camera to enable BEV)\n"
          << "  --peer-ip IPv4 --data-port PORT --control-port PORT\n"
          << "  --width N --height N --fps N --jpeg-quality N\n"
          << "  --send-raw 0|1 --stream-id N --bev-width N --bev-height N\n"
          << "  --default-output-mode bev|raw-all|front|rear|left|right\n"
          << "  --camera-stale-ms N --adaptive-illumination 0|1\n"
          << "  --thermal-degrade-c N --thermal-pause-c N\n"
          << "  --draw-vehicle 0|1 --vehicle-length-m M\n"
          << "  --vehicle-width-m M --meters-per-pixel M\n"
          << "  --pressure-log-dir PATH --pressure-test-state PATH\n"
          << "  --can-interface NAME (repeat; defaults to can10/can11)\n"
          << "  --can-stale-ms N\n";
      std::exit(0);
    } else {
      throw std::invalid_argument("unknown option: " + argument);
    }
  }
  if (options.devices.empty()) {
    options.devices.emplace_back(
        "/dev/v4l/by-id/usb-LRCP_USB3.0_LRCP_USB3.0_SN0001-video-index0");
  }
  if (options.devices.size() > 4) {
    throw std::invalid_argument("one to four cameras are supported");
  }
  if (!options.calibration_files.empty() &&
      options.calibration_files.size() != options.devices.size()) {
    throw std::invalid_argument(
        "provide either no calibrations or exactly one per camera");
  }
  if (options.raw_stream_start + static_cast<int>(options.devices.size()) > 14) {
    throw std::invalid_argument("raw camera stream IDs must remain below 14");
  }
  if (options.thermal_pause_c <= options.thermal_degrade_c) {
    throw std::invalid_argument(
        "thermal-pause-c must be greater than thermal-degrade-c");
  }
  if (options.can_interfaces.empty()) {
    options.can_interfaces = {"can10", "can11"};
  }
  return options;
}

std::string trim_lower(std::string value) {
  value.erase(value.begin(),
              std::find_if(value.begin(), value.end(), [](unsigned char ch) {
                return !std::isspace(ch);
              }));
  value.erase(std::find_if(value.rbegin(), value.rend(), [](unsigned char ch) {
                return !std::isspace(ch);
              }).base(),
              value.end());
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char ch) { return std::tolower(ch); });
  return value;
}

std::uint16_t available_raw_mask(const Options& options) {
  std::uint16_t mask = 0;
  for (std::size_t index = 0; index < options.devices.size(); ++index) {
    const int stream_id = options.raw_stream_start + static_cast<int>(index);
    if (stream_id >= 0 && stream_id < 15) {
      mask |= static_cast<std::uint16_t>(1U << stream_id);
    }
  }
  return mask;
}

std::optional<std::uint16_t> output_mask_from_name(const std::string& input,
                                                   const Options& options) {
  const std::string name = trim_lower(input);
  const std::uint16_t raw_mask = available_raw_mask(options);
  if (name == "bev") return kBevOutputBit;
  if (name == "raw-all" || name == "all") return raw_mask;
  static const std::array<const char*, 4> roles = {
      "front", "rear", "left", "right"};
  for (std::size_t index = 0; index < roles.size() &&
                              index < options.devices.size();
       ++index) {
    if (name == roles[index]) {
      return static_cast<std::uint16_t>(
          1U << (options.raw_stream_start + static_cast<int>(index)));
    }
  }
  return std::nullopt;
}

std::string output_mode_name(std::uint16_t mask, const Options& options) {
  if (mask == kBevOutputBit) return "bev";
  const std::uint16_t raw_mask = available_raw_mask(options);
  if (mask == raw_mask) return "raw-all";
  static const std::array<const char*, 4> roles = {
      "front", "rear", "left", "right"};
  for (std::size_t index = 0; index < roles.size() &&
                              index < options.devices.size();
       ++index) {
    const auto bit = static_cast<std::uint16_t>(
        1U << (options.raw_stream_start + static_cast<int>(index)));
    if (mask == bit) return roles[index];
  }
  std::ostringstream value;
  value << "mask-0x" << std::hex << std::setw(4) << std::setfill('0') << mask;
  return value.str();
}

std::uint16_t current_output_mask(const RuntimeSettings& settings) {
  return static_cast<std::uint16_t>(settings.raw_stream_mask.load() |
                                    (settings.send_bev.load()
                                         ? kBevOutputBit
                                         : 0));
}

void apply_output_mask(RuntimeSettings& settings, std::uint16_t mask) {
  const auto raw = static_cast<std::uint16_t>(mask & ~kBevOutputBit);
  settings.raw_stream_mask.store(raw);
  settings.send_raw.store(raw != 0);
  settings.send_bev.store((mask & kBevOutputBit) != 0);
}

std::uint16_t initial_output_mask(const Options& options) {
  auto parsed = output_mask_from_name(options.default_output_mode, options);
  if (!parsed) {
    throw std::invalid_argument("invalid default output mode: " +
                                options.default_output_mode);
  }
  std::uint16_t mask = *parsed;
  if (options.send_raw) mask |= available_raw_mask(options);
  return mask;
}

struct CachedFrame {
  std::vector<std::uint8_t> payload;
  std::uint64_t frame_id{0};
  std::uint64_t capture_ns{0};
  w102::MessageType type{w102::MessageType::kCameraJpeg};
  std::uint16_t flags{0};
  std::uint16_t stream_id{0};
};

class LatestFrames {
 public:
  void update(CachedFrame frame) {
    std::lock_guard<std::mutex> lock(mutex_);
    frames_[{static_cast<std::uint16_t>(frame.type), frame.stream_id}] =
        std::move(frame);
  }

  std::vector<CachedFrame> snapshots(std::uint16_t stream_mask) const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<CachedFrame> output;
    for (const auto& entry : frames_) {
      const auto& frame = entry.second;
      if (frame.stream_id < 16 &&
          (stream_mask & (1U << frame.stream_id)) != 0) {
        output.push_back(frame);
      }
    }
    return output;
  }

 private:
  mutable std::mutex mutex_;
  std::map<std::pair<std::uint16_t, std::uint16_t>, CachedFrame> frames_;
};

struct CameraSnapshot {
  cv::Mat frame;
  std::uint64_t capture_ns{0};
  std::uint64_t sequence{0};
  std::uint64_t reopen_count{0};
  bool online{false};
};

class CameraWorker {
 public:
  CameraWorker(std::string device, int width, int height, int fps)
      : device_(std::move(device)), width_(width), height_(height), fps_(fps) {}
  ~CameraWorker() { stop(); }
  CameraWorker(const CameraWorker&) = delete;
  CameraWorker& operator=(const CameraWorker&) = delete;

  void start() { thread_ = std::thread(&CameraWorker::run, this); }
  void stop() {
    stop_.store(true);
    if (thread_.joinable()) thread_.join();
  }

  CameraSnapshot snapshot(bool include_frame = true) const {
    std::lock_guard<std::mutex> lock(mutex_);
    CameraSnapshot result;
    if (include_frame) result.frame = latest_;
    result.capture_ns = capture_ns_;
    result.sequence = sequence_;
    result.reopen_count = reopen_count_;
    result.online = online_;
    return result;
  }

 private:
  bool open(cv::VideoCapture& capture) {
    capture.release();
    if (!capture.open(device_, cv::CAP_V4L2)) return false;
    capture.set(cv::CAP_PROP_FOURCC,
                cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    capture.set(cv::CAP_PROP_FRAME_WIDTH, width_);
    capture.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
    capture.set(cv::CAP_PROP_FPS, fps_);
    capture.set(cv::CAP_PROP_BUFFERSIZE, 1);
    return true;
  }

  void run() {
    cv::VideoCapture capture;
    int failures = 0;
    auto next_capture = std::chrono::steady_clock::now();
    const auto capture_period =
        std::chrono::microseconds(1000000 / std::max(1, fps_));
    while (g_running.load() && !stop_.load()) {
      if (!capture.isOpened()) {
        if (!open(capture)) {
          {
            std::lock_guard<std::mutex> lock(mutex_);
            online_ = false;
            ++reopen_count_;
          }
          std::this_thread::sleep_for(std::chrono::seconds(1));
          continue;
        }
        failures = 0;
      }
      cv::Mat frame;
      if (capture.read(frame) && !frame.empty()) {
        const auto captured = w102::monotonic_time_ns();
        {
          std::lock_guard<std::mutex> lock(mutex_);
          latest_ = frame;
          capture_ns_ = captured;
          ++sequence_;
          online_ = frame.cols == width_ && frame.rows == height_;
          failures = 0;
        }
        next_capture += capture_period;
        std::this_thread::sleep_until(next_capture);
        if (next_capture < std::chrono::steady_clock::now() - capture_period) {
          next_capture = std::chrono::steady_clock::now();
        }
        continue;
      }
      if (++failures >= 3) {
        capture.release();
        {
          std::lock_guard<std::mutex> lock(mutex_);
          online_ = false;
          ++reopen_count_;
        }
        failures = 0;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
      }
    }
    capture.release();
    std::lock_guard<std::mutex> lock(mutex_);
    online_ = false;
  }

  std::string device_;
  int width_{0};
  int height_{0};
  int fps_{0};
  mutable std::mutex mutex_;
  cv::Mat latest_;
  std::uint64_t capture_ns_{0};
  std::uint64_t sequence_{0};
  std::uint64_t reopen_count_{0};
  bool online_{false};
  std::atomic<bool> stop_{false};
  std::thread thread_;
};

class LoadMonitor {
 public:
  void update() {
    const auto wall = std::chrono::steady_clock::now();
    rusage usage{};
    getrusage(RUSAGE_SELF, &usage);
    const double process_seconds =
        usage.ru_utime.tv_sec + usage.ru_stime.tv_sec +
        (usage.ru_utime.tv_usec + usage.ru_stime.tv_usec) / 1000000.0;
    if (initialized_) {
      const double wall_seconds =
          std::chrono::duration<double>(wall - last_wall_).count();
      const double cores = std::max(1U, std::thread::hardware_concurrency());
      if (wall_seconds > 0.0) {
        process_cpu_percent_ = std::clamp(
            100.0 * (process_seconds - last_process_seconds_) /
                (wall_seconds * cores),
            0.0, 100.0);
      }
    }
    last_wall_ = wall;
    last_process_seconds_ = process_seconds;
    initialized_ = true;
    std::ifstream temperature_file("/sys/class/thermal/thermal_zone0/temp");
    double millidegrees = 0.0;
    if (temperature_file >> millidegrees) temperature_c_ = millidegrees / 1000.0;

    std::ifstream stat_file("/proc/stat");
    std::string cpu_label;
    std::uint64_t user = 0, nice = 0, system = 0, idle = 0, iowait = 0;
    std::uint64_t irq = 0, softirq = 0, steal = 0;
    if (stat_file >> cpu_label >> user >> nice >> system >> idle >> iowait >>
            irq >> softirq >> steal &&
        cpu_label == "cpu") {
      const std::uint64_t total =
          user + nice + system + idle + iowait + irq + softirq + steal;
      const std::uint64_t idle_total = idle + iowait;
      if (system_cpu_initialized_ && total > last_cpu_total_) {
        const auto total_delta = total - last_cpu_total_;
        const auto idle_delta = idle_total >= last_cpu_idle_
                                    ? idle_total - last_cpu_idle_
                                    : 0;
        system_cpu_percent_ = 100.0 *
            static_cast<double>(total_delta - std::min(total_delta, idle_delta)) /
            static_cast<double>(total_delta);
      }
      last_cpu_total_ = total;
      last_cpu_idle_ = idle_total;
      system_cpu_initialized_ = true;
    }

    std::ifstream memory_file("/proc/meminfo");
    std::string key;
    std::uint64_t value_kb = 0;
    std::string unit;
    std::uint64_t total_kb = 0;
    std::uint64_t available_kb = 0;
    while (memory_file >> key >> value_kb >> unit) {
      if (key == "MemTotal:") total_kb = value_kb;
      if (key == "MemAvailable:") available_kb = value_kb;
    }
    if (total_kb > 0) {
      memory_total_mb_ = total_kb / 1024.0;
      memory_used_mb_ = (total_kb - std::min(total_kb, available_kb)) / 1024.0;
      memory_used_percent_ = 100.0 * memory_used_mb_ / memory_total_mb_;
    }
  }
  double system_cpu_percent() const { return system_cpu_percent_; }
  double process_cpu_percent() const { return process_cpu_percent_; }
  double memory_used_percent() const { return memory_used_percent_; }
  double memory_used_mb() const { return memory_used_mb_; }
  double memory_total_mb() const { return memory_total_mb_; }
  double temperature_c() const { return temperature_c_; }

 private:
  bool initialized_{false};
  std::chrono::steady_clock::time_point last_wall_{};
  double last_process_seconds_{0.0};
  double system_cpu_percent_{0.0};
  double process_cpu_percent_{0.0};
  double temperature_c_{0.0};
  bool system_cpu_initialized_{false};
  std::uint64_t last_cpu_total_{0};
  std::uint64_t last_cpu_idle_{0};
  double memory_used_percent_{0.0};
  double memory_used_mb_{0.0};
  double memory_total_mb_{0.0};
};

class CounterRate {
 public:
  void update(std::uint64_t counter,
              const std::chrono::steady_clock::time_point& now) {
    if (!initialized_) {
      initialized_ = true;
      last_counter_ = counter;
      last_time_ = now;
      return;
    }
    const double elapsed = std::chrono::duration<double>(now - last_time_).count();
    if (elapsed >= 0.5) {
      rate_ = static_cast<double>(counter - last_counter_) / elapsed;
      last_counter_ = counter;
      last_time_ = now;
    }
  }
  double value() const { return rate_; }

 private:
  bool initialized_{false};
  std::uint64_t last_counter_{0};
  std::chrono::steady_clock::time_point last_time_{};
  double rate_{0.0};
};

struct ProcessedBev {
  std::vector<std::uint8_t> jpeg;
  std::uint64_t capture_ns{0};
  std::uint64_t generation{0};
  std::uint16_t flags{0};
  std::uint16_t active_camera_mask{0};
  std::uint16_t fresh_camera_mask{0};
  double coverage_percent{0.0};
  double process_ms{0.0};
  std::array<w102::IlluminationStats, 4> illumination{};
};

class PressureLogger {
 public:
  PressureLogger(std::string directory, std::string state_path,
                 std::uint64_t boot_id)
      : directory_(std::move(directory)),
        state_path_(std::move(state_path)),
        boot_id_(boot_id) {}

  void record(const ProcessedBev& processed, std::size_t camera_count) {
    const auto steady_now = std::chrono::steady_clock::now();
    const auto wall_now = std::chrono::system_clock::now();
    refresh_state(steady_now, wall_now);
    if (!enabled_) return;

    metrics_.update();
    const auto wall_time = std::chrono::system_clock::to_time_t(wall_now);
    std::tm local{};
    localtime_r(&wall_time, &local);
    char hour[16]{};
    char timestamp[40]{};
    std::strftime(hour, sizeof(hour), "%Y%m%d-%H", &local);
    std::strftime(timestamp, sizeof(timestamp), "%Y-%m-%dT%H:%M:%S", &local);
    const auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
                                  wall_now.time_since_epoch()) %
                              std::chrono::seconds(1);
    char timezone[8]{};
    std::strftime(timezone, sizeof(timezone), "%z", &local);

    if (!output_.is_open() || current_hour_ != hour) {
      open_hour(hour);
    }
    if (!output_.is_open()) return;

    const std::uint16_t expected_mask = static_cast<std::uint16_t>(
        camera_count >= 16 ? 0xFFFFU : ((1U << camera_count) - 1U));
    const auto fresh = processed.fresh_camera_mask;
    const int complete = (fresh & expected_mask) == expected_mask ? 1 : 0;
    const auto camera_ok = [fresh](std::size_t index) {
      return (fresh & (1U << index)) != 0 ? 1 : 0;
    };

    output_ << timestamp << '.' << std::setfill('0') << std::setw(3)
            << milliseconds.count() << timezone << ',' << complete << ','
            << camera_ok(0) << ',' << camera_ok(2) << ',' << camera_ok(1)
            << ',' << camera_ok(3) << ',' << std::fixed << std::setprecision(3)
            << metrics_.system_cpu_percent() << ','
            << metrics_.process_cpu_percent() << ','
            << metrics_.memory_used_percent() << ',' << metrics_.temperature_c()
            << '\n';
    output_.flush();
    if (!output_) {
      std::cerr << "pressure logger write failed: " << current_path_ << '\n';
      output_.close();
      current_hour_.clear();
    }
  }

 private:
  void refresh_state(
      const std::chrono::steady_clock::time_point& steady_now,
      const std::chrono::system_clock::time_point& wall_now) {
    if (state_checked_ && steady_now < next_state_check_) return;
    state_checked_ = true;
    next_state_check_ = steady_now + std::chrono::seconds(1);
    std::ifstream state(state_path_);
    std::string run_id;
    std::int64_t deadline_epoch = 0;
    state >> run_id >> deadline_epoch;
    const auto epoch = std::chrono::duration_cast<std::chrono::seconds>(
                           wall_now.time_since_epoch())
                           .count();
    bool valid_run = !directory_.empty() && !state_path_.empty() && state &&
                     deadline_epoch > epoch;
    for (const char character : run_id) {
      valid_run = valid_run &&
                  (std::isalnum(static_cast<unsigned char>(character)) ||
                   character == '-' || character == '_');
    }
    if (!valid_run) {
      enabled_ = false;
      output_.close();
      current_hour_.clear();
      return;
    }
    if (!enabled_ || run_id_ != run_id) {
      output_.close();
      current_hour_.clear();
      run_id_ = std::move(run_id);
    }
    enabled_ = true;
  }

  void open_hour(const std::string& hour) {
    output_.close();
    current_hour_.clear();
    std::error_code error;
    std::filesystem::create_directories(directory_, error);
    if (error) {
      std::cerr << "pressure logger cannot create directory: "
                << error.message() << '\n';
      return;
    }
    std::ostringstream name;
    name << "pressure-" << hour << '-' << run_id_ << "-boot" << std::hex
         << boot_id_ << ".csv";
    current_path_ = (std::filesystem::path(directory_) / name.str()).string();
    const bool new_file = !std::filesystem::exists(current_path_, error) ||
                          std::filesystem::file_size(current_path_, error) == 0;
    output_.open(current_path_, std::ios::out | std::ios::app);
    if (!output_) {
      std::cerr << "pressure logger cannot open: " << current_path_ << '\n';
      return;
    }
    current_hour_ = hour;
    if (new_file) {
      output_ << "timestamp,complete,front,left,rear,right,system_cpu_percent,"
                 "gateway_cpu_percent,memory_percent,temperature_c\n";
      output_.flush();
    }
  }

  std::string directory_;
  std::string state_path_;
  std::uint64_t boot_id_{0};
  LoadMonitor metrics_;
  bool state_checked_{false};
  bool enabled_{false};
  std::chrono::steady_clock::time_point next_state_check_{};
  std::string run_id_;
  std::string current_hour_;
  std::string current_path_;
  std::ofstream output_;
};

class LatestBev {
 public:
  void update(ProcessedBev value) {
    std::lock_guard<std::mutex> lock(mutex_);
    value_ = std::move(value);
  }
  ProcessedBev snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return value_;
  }

 private:
  mutable std::mutex mutex_;
  ProcessedBev value_;
};

bool encode_jpeg(const cv::Mat& image, int quality,
                 std::vector<std::uint8_t>& output) {
  return cv::imencode(".jpg", image, output,
                      {cv::IMWRITE_JPEG_QUALITY, quality});
}

void processing_loop(const Options& options, RuntimeSettings& settings,
                     const std::vector<std::unique_ptr<CameraWorker>>& cameras,
                     w102::BevComposer& composer, LatestBev& latest_bev,
                     std::atomic<bool>& economical_processing,
                     std::atomic<bool>& thermal_pause) {
  w102::AdaptiveIllumination illumination(cameras.size());
  std::uint64_t generation = 0;
  auto next_run = std::chrono::steady_clock::now();
  while (g_running.load()) {
    if (!settings.send_bev.load()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      next_run = std::chrono::steady_clock::now();
      continue;
    }
    const auto process_period = thermal_pause.load()
                                    ? std::chrono::milliseconds(500)
                                    : std::chrono::milliseconds(100);
    next_run += process_period;
    const auto started = std::chrono::steady_clock::now();
    const auto now_ns = w102::monotonic_time_ns();
    std::vector<cv::Mat> frames(cameras.size());
    std::vector<CameraSnapshot> snapshots(cameras.size());
    std::vector<bool> fresh_cameras(cameras.size(), false);
    std::vector<std::future<cv::Mat>> illumination_tasks(cameras.size());
    ProcessedBev result;
    result.flags = w102::kFlagJpeg | w102::kFlagValid;
    if (options.adaptive_illumination) {
      result.flags |= w102::kFlagIlluminationAdjusted;
    }
    for (std::size_t index = 0; index < cameras.size(); ++index) {
      snapshots[index] = cameras[index]->snapshot();
      const auto& snapshot = snapshots[index];
      fresh_cameras[index] = snapshot.online && !snapshot.frame.empty() &&
                             now_ns >= snapshot.capture_ns &&
                             now_ns - snapshot.capture_ns <=
                                 static_cast<std::uint64_t>(
                                     options.camera_stale_ms) *
                                     1000000ULL;
      if (!fresh_cameras[index]) {
        result.flags |= w102::kFlagPartialCameras;
        continue;
      }
      if (options.adaptive_illumination) {
        illumination_tasks[index] = std::async(
            std::launch::async, [&, index]() {
              return illumination.enhance(snapshots[index].frame, index,
                                          &result.illumination[index], true);
            });
      } else {
        frames[index] = snapshot.frame;
      }
      result.active_camera_mask |= static_cast<std::uint16_t>(1U << index);
      result.fresh_camera_mask |= static_cast<std::uint16_t>(1U << index);
      result.capture_ns = std::max(result.capture_ns, snapshot.capture_ns);
    }
    if (options.adaptive_illumination) {
      for (std::size_t index = 0; index < cameras.size(); ++index) {
        if (fresh_cameras[index]) {
          frames[index] = illumination_tasks[index].get();
        }
      }
    }
    cv::Mat coverage;
    bool synthetic_fill = false;
    const bool economical = economical_processing.load();
    cv::Mat bev = composer.compose(frames, &coverage, true,
                                   &synthetic_fill, !economical);
    if (options.adaptive_illumination && !economical) {
      double median_sum = 0.0;
      int median_count = 0;
      for (std::size_t index = 0; index < cameras.size(); ++index) {
        if ((result.active_camera_mask & (1U << index)) != 0) {
          median_sum += result.illumination[index].input_median;
          ++median_count;
        }
      }
      const bool low_light = median_count > 0 &&
                             median_sum / median_count < 35.0;
      w102::AdaptiveIllumination::enhance_local_detail(bev, low_light);
    }
    if (options.draw_vehicle) {
      w102::draw_vehicle_overlay(bev, options.vehicle_length_m,
                                 options.vehicle_width_m,
                                 options.meters_per_pixel);
    }
    if (synthetic_fill) result.flags |= w102::kFlagSyntheticFill;
    result.coverage_percent =
        100.0 * cv::countNonZero(coverage) / static_cast<double>(coverage.total());
    if (result.capture_ns == 0) result.capture_ns = now_ns;
    if (encode_jpeg(bev, settings.jpeg_quality.load(), result.jpeg)) {
      result.generation = ++generation;
      result.process_ms = std::chrono::duration<double, std::milli>(
                              std::chrono::steady_clock::now() - started)
                              .count();
      latest_bev.update(std::move(result));
    }
    std::this_thread::sleep_until(next_run);
    if (next_run < std::chrono::steady_clock::now() - process_period) {
      next_run = std::chrono::steady_clock::now();
    }
  }
}

void control_loop(const Options& options, w102::UdpFragmentSender& sender,
                  const LatestFrames& latest,
                  const std::vector<std::unique_ptr<CameraWorker>>& cameras,
                  std::uint64_t boot_id,
                  std::atomic<std::uint64_t>& hello_count,
                  RuntimeSettings& settings,
                  std::atomic<std::uint64_t>& control_ack_id) {
  const int socket_fd = socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
  if (socket_fd < 0) return;
  int reuse = 1;
  setsockopt(socket_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
  sockaddr_in bind_address{};
  bind_address.sin_family = AF_INET;
  bind_address.sin_port = htons(options.control_port);
  bind_address.sin_addr.s_addr = htonl(INADDR_ANY);
  if (bind(socket_fd, reinterpret_cast<const sockaddr*>(&bind_address),
           sizeof(bind_address)) != 0) {
    close(socket_fd);
    return;
  }
  std::array<std::uint8_t, 256> buffer{};
  while (g_running.load()) {
    pollfd descriptor{socket_fd, POLLIN, 0};
    if (poll(&descriptor, 1, 250) <= 0 ||
        (descriptor.revents & POLLIN) == 0) {
      continue;
    }
    sockaddr_in source{};
    socklen_t source_length = sizeof(source);
    const auto received = recvfrom(
        socket_fd, buffer.data(), buffer.size(), 0,
        reinterpret_cast<sockaddr*>(&source), &source_length);
    if (received <= 0) continue;
    const auto packet = w102::decode_control_packet(buffer.data(), received);
    if (!packet ||
        source.sin_addr.s_addr != sender.peer_address().sin_addr.s_addr) {
      continue;
    }
    source.sin_port = htons(packet->reply_port == 0 ? options.data_port
                                                    : packet->reply_port);
    if (packet->command == w102::ControlCommand::kHello) {
      ++hello_count;
      for (const auto& frame : latest.snapshots(packet->stream_mask)) {
        if ((frame.type == w102::MessageType::kCameraJpeg &&
             (frame.stream_id >= 15 ||
              (settings.raw_stream_mask.load() &
               static_cast<std::uint16_t>(1U << frame.stream_id)) == 0)) ||
            (frame.type == w102::MessageType::kBevJpeg &&
             !settings.send_bev.load())) {
          continue;
        }
        sender.send_frame_to(frame.payload, frame.type,
                             frame.flags | w102::kFlagReplay, frame.stream_id,
                             boot_id, frame.frame_id, frame.capture_ns, source);
      }
      continue;
    }
    bool accepted = false;
    std::string error;
    bool capture_command = false;
    std::uint16_t capture_requested_mask = 0;
    std::uint16_t capture_captured_mask = 0;
    std::uint16_t capture_missing_mask = 0;
    if (packet->command == w102::ControlCommand::kSetFeatures) {
      if ((packet->stream_mask &
           ~(w102::kFeatureRawCameras | w102::kFeatureBev)) != 0) {
        error = "unsupported feature bit";
      } else {
        std::uint16_t requested = 0;
        if ((packet->stream_mask & w102::kFeatureRawCameras) != 0) {
          requested |= available_raw_mask(options);
        }
        if ((packet->stream_mask & w102::kFeatureBev) != 0) {
          requested |= kBevOutputBit;
        }
        if (requested == 0) {
          error = "at least one output must be selected";
        } else {
          apply_output_mask(settings, requested);
          accepted = true;
        }
      }
    } else if (packet->command == w102::ControlCommand::kSetOutputMode) {
      const auto allowed = static_cast<std::uint16_t>(
          available_raw_mask(options) | kBevOutputBit);
      const auto requested = packet->stream_mask;
      if (requested == 0 || (requested & ~allowed) != 0) {
        error = "output mask selects an unavailable stream";
      } else if ((requested & kBevOutputBit) != 0 &&
                 (requested & ~kBevOutputBit) != 0) {
        error = "BEV and raw outputs are mutually exclusive";
      } else {
        apply_output_mask(settings, requested);
        accepted = true;
      }
    } else if (packet->command == w102::ControlCommand::kSetOutputFps) {
      if (packet->stream_mask < 10 || packet->stream_mask > 30) {
        error = "output fps must be in [10,30]";
      } else {
        settings.output_fps.store(packet->stream_mask);
        accepted = true;
      }
    } else if (packet->command == w102::ControlCommand::kSetJpegQuality) {
      if (packet->stream_mask < 20 || packet->stream_mask > 100) {
        error = "jpeg quality must be in [20,100]";
      } else {
        settings.jpeg_quality.store(packet->stream_mask);
        accepted = true;
      }
    } else if (packet->command == w102::ControlCommand::kCaptureRaw) {
      capture_command = true;
      std::uint16_t allowed_mask = 0;
      for (std::size_t index = 0; index < cameras.size(); ++index) {
        const auto stream_id = options.raw_stream_start +
                               static_cast<int>(index);
        if (stream_id >= 0 && stream_id < 16) {
          allowed_mask |= static_cast<std::uint16_t>(1U << stream_id);
        }
      }
      capture_requested_mask = packet->stream_mask;
      if (capture_requested_mask == 0 ||
          (capture_requested_mask & ~allowed_mask) != 0) {
        error = "capture mask selects an unavailable stream";
      } else {
        accepted = true;
        const auto requested_at_ns = w102::monotonic_time_ns();
        for (std::size_t index = 0; index < cameras.size(); ++index) {
          const auto stream_id = static_cast<std::uint16_t>(
              options.raw_stream_start + static_cast<int>(index));
          const auto stream_bit = static_cast<std::uint16_t>(1U << stream_id);
          if ((capture_requested_mask & stream_bit) == 0) continue;
          const auto snapshot = cameras[index]->snapshot();
          const bool stale = snapshot.capture_ns == 0 ||
                             requested_at_ns < snapshot.capture_ns ||
                             requested_at_ns - snapshot.capture_ns >
                                 static_cast<std::uint64_t>(
                                     options.camera_stale_ms) *
                                     1000000ULL;
          if (!snapshot.online || snapshot.frame.empty() || stale) {
            capture_missing_mask |= stream_bit;
            continue;
          }
          std::vector<std::uint8_t> encoded;
          if (!encode_jpeg(snapshot.frame, settings.jpeg_quality.load(),
                           encoded)) {
            capture_missing_mask |= stream_bit;
            continue;
          }
          const auto flags = static_cast<std::uint16_t>(
              w102::kFlagJpeg | w102::kFlagValid |
              w102::kFlagPreviewOnly);
          sender.send_frame_to(encoded, w102::MessageType::kCameraJpeg, flags,
                               stream_id, boot_id, packet->nonce,
                               snapshot.capture_ns, source);
          capture_captured_mask |= stream_bit;
        }
        capture_missing_mask |= static_cast<std::uint16_t>(
            capture_requested_mask & ~capture_captured_mask);
      }
    } else {
      error = "unsupported command";
    }
    std::ostringstream acknowledgement;
    acknowledgement << "{\"version\":1,\"nonce\":" << packet->nonce
                    << ",\"accepted\":"
                    << (accepted ? "true" : "false")
                    << ",\"send_raw\":"
                    << (settings.send_raw.load() ? "true" : "false")
                    << ",\"send_bev\":"
                    << (settings.send_bev.load() ? "true" : "false")
                    << ",\"raw_stream_mask\":"
                    << settings.raw_stream_mask.load()
                    << ",\"output_stream_mask\":"
                    << current_output_mask(settings)
                    << ",\"output_mode\":\""
                    << output_mode_name(current_output_mask(settings), options)
                    << "\""
                    << ",\"output_fps\":" << settings.output_fps.load()
                    << ",\"jpeg_quality\":"
                    << settings.jpeg_quality.load();
    if (capture_command) {
      acknowledgement << ",\"requested_mask\":" << capture_requested_mask
                      << ",\"captured_mask\":" << capture_captured_mask
                      << ",\"missing_mask\":" << capture_missing_mask;
    }
    if (!error.empty()) acknowledgement << ",\"error\":\"" << error << '"';
    acknowledgement << '}';
    const auto text = acknowledgement.str();
    const std::vector<std::uint8_t> ack(text.begin(), text.end());
    sender.send_frame_to(ack, w102::MessageType::kControlJson,
                         accepted ? w102::kFlagValid : 0, 14, boot_id,
                         ++control_ack_id, w102::monotonic_time_ns(), source);
  }
  close(socket_fd);
}

std::vector<std::uint8_t> json_bytes(const std::string& value) {
  return std::vector<std::uint8_t>(value.begin(), value.end());
}

}  // namespace

int main(int argc, char** argv) {
  std::signal(SIGINT, handle_signal);
  std::signal(SIGTERM, handle_signal);
  try {
    const Options options = parse_options(argc, argv);
    const auto boot_id = w102::random_boot_id();
    w102::UdpFragmentSender sender(options.peer_ip, options.data_port, 1400);
    LatestFrames latest;
    RuntimeSettings settings;
    const auto startup_output_mask = initial_output_mask(options);
    apply_output_mask(settings, startup_output_mask);
    settings.output_fps.store(options.fps);
    settings.jpeg_quality.store(options.jpeg_quality);
    w102::VehicleCanReader vehicle_can(options.can_interfaces,
                                       options.can_stale_ms);
    vehicle_can.start();
    std::atomic<std::uint64_t> hello_count{0};
    std::atomic<std::uint64_t> control_ack_id{0};
    std::atomic<bool> economical_processing{false};
    std::atomic<bool> thermal_pause{false};
    std::vector<std::unique_ptr<CameraWorker>> cameras;
    for (const auto& device : options.devices) {
      cameras.push_back(std::make_unique<CameraWorker>(
          device, options.width, options.height, options.fps));
      cameras.back()->start();
    }

    std::thread control_thread(control_loop, std::cref(options),
                               std::ref(sender), std::cref(latest),
                               std::cref(cameras), boot_id,
                               std::ref(hello_count), std::ref(settings),
                               std::ref(control_ack_id));

    w102::BevComposer composer(cv::Size(options.bev_width, options.bev_height));
    const bool bev_configured = !options.calibration_files.empty();
    for (const auto& calibration : options.calibration_files) {
      composer.add_camera(calibration, cv::Size(options.width, options.height));
    }
    if (bev_configured) composer.prepare();
    LatestBev latest_bev;
    if (bev_configured) {
      ProcessedBev initial_black;
      initial_black.capture_ns = w102::monotonic_time_ns();
      initial_black.flags = w102::kFlagJpeg | w102::kFlagValid |
                            w102::kFlagPartialCameras |
                            w102::kFlagRepeatedFrame;
      const cv::Mat black(options.bev_height, options.bev_width, CV_8UC3,
                          cv::Scalar(0, 0, 0));
      if (!encode_jpeg(black, options.jpeg_quality, initial_black.jpeg)) {
        throw std::runtime_error("failed to create initial black BEV");
      }
      latest_bev.update(std::move(initial_black));
    }
    std::thread processor;
    if (bev_configured && composer.ready(cameras.size())) {
      processor = std::thread(processing_loop, std::cref(options),
                              std::ref(settings), std::cref(cameras),
                              std::ref(composer), std::ref(latest_bev),
                              std::ref(economical_processing),
                              std::ref(thermal_pause));
    }

    std::array<std::uint64_t, 4> raw_frame_ids{};
    std::array<std::uint64_t, 4> raw_cached_sequences{};
    std::array<std::uint64_t, 4> raw_cached_capture_ns{};
    std::array<std::vector<std::uint8_t>, 4> raw_cached_jpegs{};
    std::array<CounterRate, 4> raw_publish_rates{};
    std::uint64_t bev_publish_id = 0;
    std::uint64_t health_frame_id = 0;
    std::uint64_t repeated_bev_count = 0;
    std::uint64_t last_bev_generation = 0;
    LoadMonitor load_monitor;
    PressureLogger pressure_logger(options.pressure_log_dir,
                                   options.pressure_test_state, boot_id);
    CounterRate bev_publish_rate;
    CounterRate bev_fusion_rate;
    bool thermal_paused = false;
    auto next_image = std::chrono::steady_clock::now();
    auto next_health = std::chrono::steady_clock::now();
    while (g_running.load()) {
      const auto now = std::chrono::steady_clock::now();
      const auto now_ns = w102::monotonic_time_ns();
      if (now >= next_image) {
        const bool preview_fallback = !bev_configured;
        if (settings.send_raw.load() || preview_fallback) {
          const std::size_t raw_count =
              preview_fallback ? std::min<std::size_t>(1, cameras.size())
                               : cameras.size();
          for (std::size_t index = 0; index < raw_count; ++index) {
            const auto stream_id = static_cast<std::uint16_t>(
                options.raw_stream_start + static_cast<int>(index));
            if (!preview_fallback &&
                (settings.raw_stream_mask.load() &
                 static_cast<std::uint16_t>(1U << stream_id)) == 0) {
              continue;
            }
            const auto snapshot = cameras[index]->snapshot();
            if (!snapshot.online || snapshot.frame.empty()) continue;
            bool repeated = snapshot.sequence == raw_cached_sequences[index];
            if (!repeated) {
              std::vector<std::uint8_t> encoded;
              if (!encode_jpeg(snapshot.frame, settings.jpeg_quality.load(),
                               encoded)) {
                continue;
              }
              raw_cached_jpegs[index] = std::move(encoded);
              raw_cached_sequences[index] = snapshot.sequence;
              raw_cached_capture_ns[index] = snapshot.capture_ns;
            }
            if (raw_cached_jpegs[index].empty()) continue;
            const auto frame_id = ++raw_frame_ids[index];
            auto flags = static_cast<std::uint16_t>(
                w102::kFlagJpeg | w102::kFlagValid | w102::kFlagPreviewOnly);
            if (repeated) flags |= w102::kFlagRepeatedFrame;
            latest.update({raw_cached_jpegs[index], frame_id,
                           raw_cached_capture_ns[index],
                           w102::MessageType::kCameraJpeg, flags, stream_id});
            sender.send_frame(raw_cached_jpegs[index],
                              w102::MessageType::kCameraJpeg, flags, stream_id,
                              boot_id, frame_id, raw_cached_capture_ns[index]);
          }
        }

        if (settings.send_bev.load() && bev_configured) {
          auto processed = latest_bev.snapshot();
          if (!processed.jpeg.empty()) {
            std::uint16_t flags = processed.flags;
            if (processed.generation == last_bev_generation) {
              flags |= w102::kFlagRepeatedFrame;
              ++repeated_bev_count;
            }
            last_bev_generation = processed.generation;
            constexpr std::uint16_t bev_stream_id = 15;
            const auto publish_id = ++bev_publish_id;
            latest.update({processed.jpeg, publish_id, processed.capture_ns,
                           w102::MessageType::kBevJpeg, flags, bev_stream_id});
            sender.send_frame(processed.jpeg, w102::MessageType::kBevJpeg,
                              flags, bev_stream_id, boot_id, publish_id,
                              processed.capture_ns);
            pressure_logger.record(processed, cameras.size());
          }
        }
        const auto image_period = std::chrono::microseconds(
            1000000 / std::max(1, settings.output_fps.load()));
        next_image += image_period;
        if (next_image < now - image_period) next_image = now + image_period;
      }

      if (now >= next_health) {
        load_monitor.update();
        if (load_monitor.temperature_c() >= options.thermal_pause_c) {
          thermal_paused = true;
        } else if (load_monitor.temperature_c() <=
                   options.thermal_degrade_c) {
          thermal_paused = false;
        }
        thermal_pause.store(thermal_paused);
        const bool economical =
            load_monitor.system_cpu_percent() > 60.0 ||
            load_monitor.temperature_c() >= options.thermal_degrade_c;
        economical_processing.store(economical);
        const auto processed = latest_bev.snapshot();
        const auto vehicle_status = vehicle_can.snapshot();
        bev_publish_rate.update(bev_publish_id, now);
        bev_fusion_rate.update(processed.generation, now);
        double selected_raw_send_fps = 0.0;
        bool has_selected_raw_rate = false;
        for (std::size_t index = 0; index < raw_publish_rates.size(); ++index) {
          raw_publish_rates[index].update(raw_frame_ids[index], now);
          const auto stream_id = static_cast<std::uint16_t>(
              options.raw_stream_start + static_cast<int>(index));
          if ((settings.raw_stream_mask.load() &
               static_cast<std::uint16_t>(1U << stream_id)) == 0) {
            continue;
          }
          const double rate = raw_publish_rates[index].value();
          selected_raw_send_fps = has_selected_raw_rate
                                      ? std::min(selected_raw_send_fps, rate)
                                      : rate;
          has_selected_raw_rate = true;
        }
        const double output_send_fps = settings.send_bev.load()
                                           ? bev_publish_rate.value()
                                           : selected_raw_send_fps;
        const double bev_age_ms =
            processed.capture_ns == 0 || now_ns < processed.capture_ns
                ? -1.0
                : (now_ns - processed.capture_ns) / 1000000.0;
        std::ostringstream health;
        health << "{\"version\":1,\"boot_id\":" << boot_id
               << ",\"camera_count\":" << cameras.size()
               << ",\"bev_configured\":"
               << (bev_configured ? "true" : "false")
               << ",\"send_raw\":"
               << (settings.send_raw.load() ? "true" : "false")
               << ",\"send_bev\":"
               << (settings.send_bev.load() ? "true" : "false")
               << ",\"raw_stream_mask\":"
               << settings.raw_stream_mask.load()
               << ",\"output_stream_mask\":"
               << current_output_mask(settings)
               << ",\"output_mode\":\""
               << output_mode_name(current_output_mask(settings), options)
               << "\""
               << ",\"output_fps\":" << settings.output_fps.load()
               << ",\"bev_publish_id\":" << bev_publish_id
               << ",\"bev_generation\":" << processed.generation
               << ",\"bev_repeated_count\":" << repeated_bev_count
               << ",\"bev_active_camera_mask\":"
               << processed.active_camera_mask
               << ",\"bev_fresh_camera_mask\":"
               << processed.fresh_camera_mask
               << ",\"bev_coverage_percent\":"
               << processed.coverage_percent
               << ",\"bev_process_ms\":" << processed.process_ms
               << ",\"bev_age_ms\":" << bev_age_ms
               << ",\"bev_send_fps\":" << bev_publish_rate.value()
               << ",\"bev_fusion_fps\":" << bev_fusion_rate.value()
               << ",\"raw_send_fps\":["
               << raw_publish_rates[0].value() << ','
               << raw_publish_rates[1].value() << ','
               << raw_publish_rates[2].value() << ','
               << raw_publish_rates[3].value() << ']'
               << ",\"output_send_fps\":" << output_send_fps
               << ",\"image_processing_fps\":"
               << (settings.send_bev.load() ? bev_fusion_rate.value()
                                            : output_send_fps)
               << ",\"system_cpu_percent\":"
               << load_monitor.system_cpu_percent()
               << ",\"cpu_total_percent\":"
               << load_monitor.process_cpu_percent()
               << ",\"process_cpu_percent\":"
               << load_monitor.process_cpu_percent()
               << ",\"gateway_cpu_percent_total_capacity\":"
               << load_monitor.process_cpu_percent()
               << ",\"memory_used_percent\":"
               << load_monitor.memory_used_percent()
               << ",\"memory_used_mb\":"
               << load_monitor.memory_used_mb()
               << ",\"memory_total_mb\":"
               << load_monitor.memory_total_mb()
               << ",\"temperature_c\":" << load_monitor.temperature_c()
               << ",\"economical_processing\":"
               << (economical ? "true" : "false")
               << ",\"thermal_limited\":"
               << (thermal_paused ? "true" : "false")
               << ",\"hello_count\":" << hello_count.load()
               << ",\"vehicle_status\":"
               << w102::vehicle_can_status_json(
                      vehicle_status, now_ns, vehicle_can.stale_ms())
               << ",\"cameras\":[";
        for (std::size_t index = 0; index < cameras.size(); ++index) {
          static constexpr std::array<const char*, 4> roles{
              "front", "rear", "left", "right"};
          if (index != 0) health << ',';
          const auto status = cameras[index]->snapshot(false);
          const double age_ms =
              status.capture_ns == 0 || now_ns < status.capture_ns
                  ? -1.0
                  : (now_ns - status.capture_ns) / 1000000.0;
          health << "{\"index\":" << index
                 << ",\"role\":\"" << roles[index] << "\""
                 << ",\"online\":" << (status.online ? "true" : "false")
                 << ",\"sequence\":" << status.sequence
                 << ",\"age_ms\":" << age_ms
                 << ",\"reopens\":" << status.reopen_count
                 << ",\"luma_median\":"
                 << processed.illumination[index].input_median
                 << ",\"luma_p99\":"
                 << processed.illumination[index].input_p99
                 << ",\"gamma\":"
                 << processed.illumination[index].applied_gamma << '}';
        }
        health << "]}";
        const auto payload = json_bytes(health.str());
        sender.send_frame(payload, w102::MessageType::kHealthJson,
                          w102::kFlagValid, 14, boot_id, ++health_frame_id,
                          now_ns);
        next_health += std::chrono::milliseconds(100);
        if (next_health < now - std::chrono::milliseconds(100)) {
          next_health = now + std::chrono::milliseconds(100);
        }
      }
      const auto next_deadline = std::min(next_image, next_health);
      std::this_thread::sleep_until(next_deadline);
    }

    for (auto& camera : cameras) camera->stop();
    vehicle_can.stop();
    if (processor.joinable()) processor.join();
    if (control_thread.joinable()) control_thread.join();
  } catch (const std::exception& error) {
    std::cerr << "fatal: " << error.what() << '\n';
    g_running.store(false);
    return 1;
  }
  return 0;
}
