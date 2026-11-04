#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "sensor_driver_msgs/msg/chassis_can_recived.hpp"
#include "sensor_driver_msgs/msg/chassis_can_sended.hpp"
#include <string>
#include <sstream>
#include <utility>
#include <chrono>
#include <cmath>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/file.h>
#include <bitset>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <memory>
#include <stdexcept>
#include <vector>
#include <algorithm>
#include <array>
#include <functional>

using namespace std;

namespace
{
class StableCanLock
{
public:
  explicit StableCanLock(const std::string & interface_name)
  {
    uint64_t hash = 14695981039346656037ULL;
    for (const unsigned char value : interface_name) {
      hash ^= value;
      hash *= 1099511628211ULL;
    }
    const std::string runtime_dir = "/run/user/" + std::to_string(getuid());
    const std::string lock_dir = access(runtime_dir.c_str(), W_OK) == 0 ? runtime_dir : "/tmp";
    std::ostringstream path;
    path << lock_dir << "/white_ws_" << getuid() << "_can_" << std::hex <<
      std::setw(16) << std::setfill('0') << hash << ".lock";
    const std::string lock_path = path.str();
    fd_ = open(lock_path.c_str(), O_CREAT | O_CLOEXEC | O_RDWR | O_NOFOLLOW, 0600);
    if (fd_ < 0 || flock(fd_, LOCK_EX | LOCK_NB) != 0) {
      const int error_number = errno;
      if (fd_ >= 0) {
        close(fd_);
        fd_ = -1;
      }
      throw std::runtime_error(
              "cannot lock SocketCAN interface " + interface_name + " at " + lock_path + ": " +
              std::strerror(error_number));
    }
  }

  ~StableCanLock()
  {
    if (fd_ >= 0) {
      close(fd_);
    }
  }

  StableCanLock(const StableCanLock &) = delete;
  StableCanLock & operator=(const StableCanLock &) = delete;

private:
  int fd_{-1};
};

class ReconnectingCanSocket
{
public:
  using Clock = std::chrono::steady_clock;
  using LogCallback = std::function<void(const std::string &)>;

  ReconnectingCanSocket(
    std::string interface_name, LogCallback error_callback, LogCallback recovered_callback)
  : interface_name_(std::move(interface_name)),
    instance_lock_(interface_name_),
    error_callback_(std::move(error_callback)),
    recovered_callback_(std::move(recovered_callback)),
    last_open_attempt_(Clock::now() - kReconnectPeriod),
    last_error_report_(Clock::now())
  {
    if (interface_name_.empty() || interface_name_.size() >= IFNAMSIZ) {
      throw std::invalid_argument("invalid SocketCAN interface name: " + interface_name_);
    }
  }

  ~ReconnectingCanSocket()
  {
    close_socket();
  }

  ReconnectingCanSocket(const ReconnectingCanSocket &) = delete;
  ReconnectingCanSocket & operator=(const ReconnectingCanSocket &) = delete;

  bool service()
  {
    const auto current = Clock::now();
    if (fd_ >= 0) {
      if (current - last_valid_feedback_ >= kFeedbackTimeout) {
        disconnect("no valid chassis feedback for 2000 ms; reconnecting", current);
        return false;
      }
      report_offline_if_due(current);
      return true;
    }
    if (current - last_open_attempt_ < kReconnectPeriod) {
      report_offline_if_due(current);
      return false;
    }
    last_open_attempt_ = current;
    return open_socket(current);
  }

  bool is_connected() const
  {
    return fd_ >= 0;
  }

  int fd() const
  {
    return fd_;
  }

  void mark_valid_feedback()
  {
    last_valid_feedback_ = Clock::now();
    if (offline_) {
      offline_ = false;
      if (recovered_callback_) {
        recovered_callback_("valid chassis feedback restored on " + interface_name_);
      }
    }
  }

  void handle_io_error(const std::string & operation, const int error_number)
  {
    disconnect(
      operation + " failed: " + std::string(std::strerror(error_number)) + "; reconnecting",
      Clock::now());
  }

  bool take_connection_reset()
  {
    return std::exchange(reset_pending_, false);
  }

private:
  static constexpr std::chrono::seconds kReconnectPeriod{1};
  static constexpr std::chrono::seconds kFeedbackTimeout{2};
  static constexpr std::chrono::seconds kErrorReportPeriod{5};

  bool open_socket(const Clock::time_point current)
  {
    const int candidate = socket(PF_CAN, SOCK_RAW | SOCK_CLOEXEC, CAN_RAW);
    if (candidate < 0) {
      set_offline_reason("cannot create SocketCAN socket: " + std::string(std::strerror(errno)));
      report_offline_if_due(current);
      return false;
    }

    struct ifreq interface_index_request {};
    std::strncpy(interface_index_request.ifr_name, interface_name_.c_str(), IFNAMSIZ - 1);
    if (ioctl(candidate, SIOCGIFINDEX, &interface_index_request) < 0) {
      const int error_number = errno;
      close(candidate);
      set_offline_reason(
        "interface is unavailable: " + std::string(std::strerror(error_number)));
      report_offline_if_due(current);
      return false;
    }
    struct ifreq interface_flags_request {};
    std::strncpy(interface_flags_request.ifr_name, interface_name_.c_str(), IFNAMSIZ - 1);
    if (ioctl(candidate, SIOCGIFFLAGS, &interface_flags_request) < 0) {
      const int error_number = errno;
      close(candidate);
      set_offline_reason(
        "cannot read interface flags: " + std::string(std::strerror(error_number)));
      report_offline_if_due(current);
      return false;
    }
    if ((interface_flags_request.ifr_flags & IFF_UP) == 0) {
      close(candidate);
      set_offline_reason("interface exists but is DOWN; waiting without configuring it");
      report_offline_if_due(current);
      return false;
    }

    struct sockaddr_can address {};
    address.can_family = AF_CAN;
    address.can_ifindex = interface_index_request.ifr_ifindex;
    constexpr canid_t kStandardDataFrameMask = CAN_SFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG;
    const std::array<can_filter, 7> filters{{
      {0x352, kStandardDataFrameMask}, {0x354, kStandardDataFrameMask},
      {0x356, kStandardDataFrameMask}, {0x358, kStandardDataFrameMask},
      {0x360, kStandardDataFrameMask}, {0x362, kStandardDataFrameMask},
      {0x364, kStandardDataFrameMask}}};
    if (setsockopt(
        candidate, SOL_CAN_RAW, CAN_RAW_FILTER, filters.data(),
        static_cast<socklen_t>(filters.size() * sizeof(can_filter))) < 0 ||
      bind(candidate, reinterpret_cast<struct sockaddr *>(&address), sizeof(address)) < 0)
    {
      const int error_number = errno;
      close(candidate);
      set_offline_reason("cannot prepare interface: " + std::string(std::strerror(error_number)));
      report_offline_if_due(current);
      return false;
    }
    const int flags = fcntl(candidate, F_GETFL, 0);
    if (flags < 0 || fcntl(candidate, F_SETFL, flags | O_NONBLOCK) < 0) {
      const int error_number = errno;
      close(candidate);
      set_offline_reason(
        "cannot configure non-blocking socket: " + std::string(std::strerror(error_number)));
      report_offline_if_due(current);
      return false;
    }

    fd_ = candidate;
    reset_pending_ = true;
    last_valid_feedback_ = current;
    set_offline_reason("interface opened; waiting for valid chassis feedback");
    return true;
  }

  void disconnect(const std::string & reason, const Clock::time_point current)
  {
    set_offline_reason(reason);
    close_socket();
    last_open_attempt_ = current;
    report_offline_if_due(current);
  }

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
      error_callback_(interface_name_ + ": " + offline_reason_);
    }
  }

  void close_socket()
  {
    if (fd_ >= 0) {
      close(fd_);
      fd_ = -1;
      reset_pending_ = true;
    }
  }

  std::string interface_name_;
  StableCanLock instance_lock_;
  LogCallback error_callback_;
  LogCallback recovered_callback_;
  int fd_{-1};
  Clock::time_point last_open_attempt_;
  Clock::time_point last_valid_feedback_{};
  Clock::time_point last_error_report_;
  bool offline_{true};
  bool reset_pending_{false};
  std::string offline_reason_{"interface is not connected"};
};
}  // namespace

/*驱动底盘CAN总线协议反馈端（含有反馈信息发布和订阅控制指令功能）*/
class chassis_can : public rclcpp::Node{
 public:

  // 构造函数,有一个参数为节点名称
  explicit chassis_can(std::string name) : Node(name){

    /*声明参数*/
    this->declare_parameter("chassis_can_name", "can0");
    this->declare_parameter("torque_coefficient", 1.0); 
    this->declare_parameter("lr_signal", 1); 
    this->declare_parameter("control_methon_all_flag", false); 
    this->declare_parameter("control_methon_ssb_flag", true); 
    this->declare_parameter("danger_subscribe_topic", "/Control_msgs/danger_flag"); 
    this->declare_parameter("speed_subscribe_topic", "/Danger_msgs/danger_flag"); 
    this->declare_parameter("steerAngle_subscribe_topic", "/Control_msgs/steerAngle"); 
    this->declare_parameter("brake_subscribe_topic", "/Control_msgs/brakeRate"); 
    this->declare_parameter("all_subscribe_topic", "/Control_msgs/chassisCan"); 
    this->declare_parameter("rawData_publisher_topic", "/Sensor_msgs/chassis_can/rawData"); 
    this->declare_parameter("speed_publisher_topic", "/Sensor_msgs/chassis_can/speed"); 
    this->declare_parameter("steerAngle_publisher_topic", "/Sensor_msgs/chassis_can/steerAngle"); 
    this->declare_parameter("brake_publisher_topic", "/Sensor_msgs/chassis_can/brakeRate"); 
    this->declare_parameter("soc_publisher_topic", "/Sensor_msgs/chassis_can/SOC"); 
    this->declare_parameter("all_publisher_topic", "/Sensor_msgs/chassis_can/chassisCan"); 
    this->declare_parameter("command_timeout_sec", 0.5);
    this->declare_parameter("feedback_timeout_sec", 0.2);
    this->declare_parameter("enable_tx", false);
    this->declare_parameter("allow_arm_with_mechanical_estop", false);
    this->declare_parameter("stationary_speed_raw", 10);
    this->declare_parameter("vcu_handshake_grace_sec", 0.5);

    /*获取参数*/
    this->get_parameter("chassis_can_name", CAN_name);
    this->get_parameter("torque_coefficient", torque_coefficient);
    this->get_parameter("lr_signal", lr_signal);
    this->get_parameter("control_methon_all_flag", control_methon_all_flag);
    if(control_methon_all_flag)control_methon_ssb_flag=false;
    this->get_parameter("control_methon_ssb_flag", control_methon_ssb_flag);
    if(control_methon_ssb_flag)control_methon_all_flag=false;
    this->get_parameter("danger_subscribe_topic", danger_subscribe_topic);
    this->get_parameter("speed_subscribe_topic", speed_subscribe_topic);
    this->get_parameter("steerAngle_subscribe_topic", steerAngle_subscribe_topic);
    this->get_parameter("brake_subscribe_topic", brake_subscribe_topic);
    this->get_parameter("all_subscribe_topic", all_subscribe_topic);
    this->get_parameter("rawData_publisher_topic", rawData_publisher_topic);
    this->get_parameter("speed_publisher_topic", speed_publisher_topic);
    this->get_parameter("steerAngle_publisher_topic", steerAngle_publisher_topic);
    this->get_parameter("brake_publisher_topic", brake_publisher_topic);
    this->get_parameter("soc_publisher_topic", soc_publisher_topic);
    this->get_parameter("all_publisher_topic", all_publisher_topic);
    this->get_parameter("command_timeout_sec", command_timeout_sec_);
    this->get_parameter("feedback_timeout_sec", feedback_timeout_sec_);
    this->get_parameter("enable_tx", enable_tx_);
    this->get_parameter(
      "allow_arm_with_mechanical_estop", allow_arm_with_mechanical_estop_);
    this->get_parameter("stationary_speed_raw", stationary_speed_raw_);
    this->get_parameter("vcu_handshake_grace_sec", vcu_handshake_grace_sec_);
    if (!std::isfinite(command_timeout_sec_) || command_timeout_sec_ <= 0.0) {
      throw std::invalid_argument("command_timeout_sec must be finite and greater than zero");
    }
    if (!std::isfinite(feedback_timeout_sec_) || feedback_timeout_sec_ <= 0.0) {
      throw std::invalid_argument("feedback_timeout_sec must be finite and greater than zero");
    }
    if (stationary_speed_raw_ < 0 || stationary_speed_raw_ > 2047) {
      throw std::invalid_argument("stationary_speed_raw must be in [0, 2047]");
    }
    if (!std::isfinite(vcu_handshake_grace_sec_) || vcu_handshake_grace_sec_ <= 0.0) {
      throw std::invalid_argument("vcu_handshake_grace_sec must be finite and greater than zero");
    }

    std::stringstream ss;
    ss.str("");
    ss << "\nCAN_name = " << CAN_name;
    ss << "\ntorque_coefficient = " << torque_coefficient;
    ss << "\nlr_signal = " << lr_signal;
    ss << "\ncontrol_methon_all_flag = " << control_methon_all_flag;
    ss << "\ncontrol_methon_ssb_flag = " << control_methon_ssb_flag;
    ss << "\ndanger_subscribe_topic = " << danger_subscribe_topic;
    ss << "\nspeed_subscribe_topic = " << speed_subscribe_topic;
    ss << "\nsteerAngle_subscribe_topic = " << steerAngle_subscribe_topic;
    ss << "\nbrake_subscribe_topic = " << brake_subscribe_topic;
    ss << "\nall_subscribe_topic = " << all_subscribe_topic;
    ss << "\nrawData_publisher_topic = " << rawData_publisher_topic;
    ss << "\nspeed_publisher_topic = " << speed_publisher_topic;
    ss << "\nsteerAngle_publisher_topic = " << steerAngle_publisher_topic;
    ss << "\nbrake_publisher_topic = " << brake_publisher_topic;
    ss << "\nsoc_publisher_topic = " << soc_publisher_topic;
    ss << "\nall_publisher_topic = " << all_publisher_topic;
    std::string print_str = ss.str();
    RCLCPP_INFO(this->get_logger(), "%s", print_str.c_str());

    /*创建发布函数*/
    rawData_publisher_ = this->create_publisher<std_msgs::msg::String>(rawData_publisher_topic, 10);
    speed_publisher_ = this->create_publisher<std_msgs::msg::Float32>(speed_publisher_topic, 10);
    steerAngle_publisher_ = this->create_publisher<std_msgs::msg::Float32>(steerAngle_publisher_topic, 10);
    brake_publisher_ = this->create_publisher<std_msgs::msg::Float32>(brake_publisher_topic, 10);
    soc_publisher_ = this->create_publisher<std_msgs::msg::Int16>(soc_publisher_topic, 10);
    all_publisher_ = this->create_publisher<sensor_driver_msgs::msg::ChassisCanRecived>(all_publisher_topic, 10);
    armed_publisher_ = this->create_publisher<std_msgs::msg::Bool>(
      "/chassis_can/armed", rclcpp::QoS(1).reliable().transient_local());

    /*创建订阅函数*/
    danger_subscribe_ = this->create_subscription<std_msgs::msg::Int16>(danger_subscribe_topic, 10, std::bind(&chassis_can::command_callback_chassis_danger, this, std::placeholders::_1));
    // speed_subscribe_ = this->create_subscription<std_msgs::msg::Int16>(speed_subscribe_topic, 10, std::bind(&chassis_can::command_callback_chassis_speed, this, std::placeholders::_1));
    // steerAngle_subscribe_ = this->create_subscription<std_msgs::msg::Float32>(steerAngle_subscribe_topic, 10, std::bind(&chassis_can::command_callback_chassis_steerAngle, this, std::placeholders::_1));
    // brake_subscribe_ = this->create_subscription<std_msgs::msg::Float32>(brake_subscribe_topic, 10, std::bind(&chassis_can::command_callback_chassis_brake, this, std::placeholders::_1));
    all_subscribe_ = this->create_subscription<sensor_driver_msgs::msg::ChassisCanSended>(all_subscribe_topic,10,std::bind(&chassis_can::command_callback_chassis_can_sended, this, std::placeholders::_1));
    arm_service_ = this->create_service<std_srvs::srv::SetBool>(
      "/chassis_can/arm",
      std::bind(
        &chassis_can::handle_arm_request, this, std::placeholders::_1,
        std::placeholders::_2));

    /*初始化CAN数组信号*/
    can_vec.reserve(7);
    frame.can_dlc = 8;
    frame.can_id = 0x351;
    for(int i = 0; i < 8; i ++) frame.data[i] = 0x00;
    can_vec.push_back(frame);
    frame.can_id = 0x353;
    for(int i = 0; i < 8; i ++) frame.data[i] = 0x00;
    can_vec.push_back(frame);
    frame.can_id = 0x355;
    for(int i = 0; i < 8; i ++) frame.data[i] = 0x00;
    can_vec.push_back(frame);
    frame.can_id = 0x357;
    for(int i = 0; i < 8; i ++) frame.data[i] = 0x00;
    can_vec.push_back(frame);
    frame.can_id = 0x359;
    for(int i = 0; i < 8; i ++) frame.data[i] = 0x00;
    can_vec.push_back(frame);
    frame.can_id = 0x361;
    for(int i = 0; i < 8; i ++) frame.data[i] = 0x00;
    can_vec.push_back(frame);
    frame.can_id = 0x363;
    for(int i = 0; i < 8; i ++) frame.data[i] = 0x00;
    can_vec.push_back(frame);

    /*初始化计数器*/
    IDUVCU_Ctrl_RollCnt = 0;
    // Safe power-up state: manual mode, neutral gear, parking brake and service brake applied.
    IDUVCU_Ctrl_WorkMod = 0x0;
    IDUVCU_Ctrl_ESTOP = 0x0; //取消紧急制动状态
    IDUVCU_Ctrl_ReStop = 0x0; //停止重置制动状态
    IDUMCU_Ctrl_WorkMod = 0x1; //车速控制为速度模式
    IDUEPB_Ctrl_ReqPark = 0x1;
    IDUBCM_Ctrl_HeadLight = 0x0;
    IDUMCU_Ctrl_RND = 0x0;
    IDUVCU_Ctrl_Checksum = 0;
    IDUMCU_Ctrl_TgtSped = 0;
    IDUMCU_Ctrl_TgtAcc = 0;
    IDUMCU_Ctrl_TgtTorq = 0;
    IDUEPS_Ctrl_TgtAngle = 0;
    IDUEHB_Ctrl_BrakMod = 2;
    IDUEHB_Ctrl_TgtPressure = 10;
    IDUEHB_Ctrl_TgtDece = 0;
    IDUBMS_Ctrl_Power = 0;
    IDUBCM_Ctrl_LeftFlash = 0;
    IDUBCM_Ctrl_RightFlash = 0;
    IDUBCM_Ctrl_BackLight = 0;
    IDUBCM_Ctrl_BrakeLight = 0;
    IDUBCM_Ctrl_Siren = 0;
    IDUBCM_Ctrl_Voice = 0;
    IDUBCM_Ctrl_DoubleFlash = 0;

    can_socket_ = std::make_unique<ReconnectingCanSocket>(
      CAN_name,
      [this](const std::string & message) {
        RCLCPP_ERROR(this->get_logger(), "Chassis CAN offline: %s", message.c_str());
      },
      [this](const std::string & message) {
        RCLCPP_INFO(this->get_logger(), "Chassis CAN recovered: %s", message.c_str());
      });

    /*启动成功*/
    RCLCPP_INFO(this->get_logger(), "节点已启动：%s.", name.c_str());

    using namespace std::literals::chrono_literals;
    timer_ = this->create_wall_timer(
        1ms, std::bind(&chassis_can::data_callback, this));
    if (enable_tx_) {
      timerSend_ = this->create_wall_timer(
          10ms, std::bind(&chassis_can::data_sendCAN, this));
      RCLCPP_WARN(
        this->get_logger(),
        "CAN transmit path is enabled but DISARMED; /chassis_can/arm is still required");
    } else {
      RCLCPP_INFO(this->get_logger(), "CAN receive-only mode: no write timer was created");
    }
    publish_armed_state();
  }

 private:
  /*声明CAN相关变量*/
  std::unique_ptr<ReconnectingCanSocket> can_socket_;
  ssize_t nbytes{0};
  struct can_frame frame;
  std::string CAN_name;
  std::vector<can_frame> can_vec;
  int danger_flag = 0;
  double command_timeout_sec_{0.5};
  double feedback_timeout_sec_{0.2};
  bool enable_tx_{false};
  bool allow_arm_with_mechanical_estop_{false};
  bool armed_{false};
  int stationary_speed_raw_{10};
  double vcu_handshake_grace_sec_{0.5};
  bool have_command_{false};
  std::chrono::steady_clock::time_point last_command_{std::chrono::steady_clock::now()};
  std::chrono::steady_clock::time_point armed_since_{std::chrono::steady_clock::now()};
  std::array<bool, 5> critical_feedback_seen_{{false, false, false, false, false}};
  std::array<std::chrono::steady_clock::time_point, 5> critical_feedback_time_{};
  
  //bool have_rev_controlCmd=false;

  /*声明时间循环、发布者和订阅者*/
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timerSend_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rawData_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr speed_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steerAngle_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr brake_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr soc_publisher_;
  rclcpp::Publisher<sensor_driver_msgs::msg::ChassisCanRecived>::SharedPtr all_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr armed_publisher_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr danger_subscribe_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr speed_subscribe_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr steerAngle_subscribe_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr brake_subscribe_;
  rclcpp::Subscription<sensor_driver_msgs::msg::ChassisCanSended>::SharedPtr all_subscribe_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr arm_service_;

  /*CAN报文信息原始值*/
    /*控制部分*/
  unsigned int IDUVCU_Ctrl_WorkMod{}, IDUVCU_Ctrl_ESTOP{}, IDUVCU_Ctrl_ReStop{}, IDUVCU_Ctrl_RollCnt{}, IDUVCU_Ctrl_Checksum{};//地址0x351
  unsigned int IDUMCU_Ctrl_WorkMod{}, IDUMCU_Ctrl_RND{};//地址0x353
  int IDUMCU_Ctrl_TgtSped{}, IDUMCU_Ctrl_TgtAcc{}, IDUMCU_Ctrl_TgtTorq{};//地址0x353
  unsigned int IDUEPB_Ctrl_ReqPark{};//地址0x355
  int IDUEPS_Ctrl_TgtAngle{};//地址0x357
  unsigned int IDUEHB_Ctrl_BrakMod{}, IDUEHB_Ctrl_TgtPressure{};//地址0x359
  int IDUEHB_Ctrl_TgtDece{};//地址0x359
  unsigned int IDUBMS_Ctrl_Power{};//地址0x361
  unsigned int IDUBCM_Ctrl_HeadLight{}, IDUBCM_Ctrl_LeftFlash{}, IDUBCM_Ctrl_RightFlash{}, IDUBCM_Ctrl_BackLight{}, IDUBCM_Ctrl_BrakeLight{}, IDUBCM_Ctrl_Siren{}, IDUBCM_Ctrl_Voice{}, IDUBCM_Ctrl_DoubleFlash{};//地址0x363

    /*反馈部分*/
  unsigned int VCUIDU_Fdbk_WorkMod{}, VCUIDU_Fdbk_ModelStop{}, VCUIDU_Fdbk_Power{}, VCUIDU_Fdbk_Erro{}, VCU_Fdbk_SoftVer{}, VCU_Fdbk_RollCnt{}, VCUIDU_Fdbk_Checksum{}; //地址0x352
  unsigned int MCUIDU_Fdbk_RND{}, MCUIDU_Fdbk_RealTorq{}, MCUIDU_Fdbk_RealRpm{}, MCUIDU_Fdbk_ErroLevel{}, MCUIDU_Fdbk_Fault{};//地址0x354
  int MCUIDU_Fdbk_RealSped{}, MCUIDU_Fdbk_RealAcc{};//地址0x354，带有正负
  unsigned int EPBIDU_Fdbk_ParkSta{}, EPBIDU_Fdbk_ErroLevel{}, EPBIDU_Fdbk_Fault{};//地址0x356
  unsigned int EPSIDU_Fdbk_WorkSta{}, EPSIDU_Fdbk_ErroLevel{}, EPSIDU_Fdbk_Fault{};//地址0x358
  int EPSIDU_Fdbk_RealAngle{};//地址0x358，带有正负
  unsigned int EHBIDU_Fdbk_RealPressure{}, EHBIDU_Fdbk_ErroLevel{}, EHBIDU_Fdbk_Fault{};//地址0x360
  int EHBIDU_Fdbk_RealDece{};//地址0x360，带有正负
  unsigned int BMSIDU_Fdbk_CHCSta{}, BMSIDU_Fdbk_SOC{}, BMSIDU_Fdbk_BusV{}, BMSIDU_Fdbk_ErroLevel{}, BMSIDU_Fdbk_Fault{};//地址0x362
  int BMSIDU_Fdbk_BusA{};//地址0x362，带有正负
  unsigned int BCMIDU_Fdbk_HeadLight{}, BCMIDU_Fdbk_TurnFlash{}, BCMIDU_Fdbk_BackLight{}, BCMIDU_Fdbk_BrakeLight{}, BCMIDU_Fdbk_Siren{}, BCMIDU_Fdbk_Voice{}, BCMIDU_Fdbk_DoubleFlash{};//地址0x364
  unsigned int check_sum_counter{};

  /*底盘数据处理后值*/
  float torque_coefficient;
  int lr_signal{}, pn_signal{};
  bool control_methon_all_flag,control_methon_ssb_flag;
  float speed_publisher_float = 0.0;
  float steerAngle_publisher_float = 0.0;
  std::string danger_subscribe_topic,speed_subscribe_topic,steerAngle_subscribe_topic,brake_subscribe_topic,all_subscribe_topic;
  std::string rawData_publisher_topic,speed_publisher_topic,steerAngle_publisher_topic,brake_publisher_topic,soc_publisher_topic,all_publisher_topic;
  std_msgs::msg::String rawData_publisher_message;
  std_msgs::msg::Float32 speed_publisher_message;
  std_msgs::msg::Float32 steerAngle_publisher_message;
  std_msgs::msg::Float32 brake_publisher_message;
  std_msgs::msg::Int16 soc_publisher_message;
  sensor_driver_msgs::msg::ChassisCanRecived all_publisher_message;

  void publish_armed_state() {
    std_msgs::msg::Bool message;
    message.data = armed_;
    armed_publisher_->publish(message);
  }

  void disarm(const char * reason) {
    if (armed_) {
      armed_ = false;
      publish_armed_state();
      RCLCPP_ERROR(this->get_logger(), "CAN transmit path DISARMED: %s", reason);
    }
  }

  bool critical_feedback_fresh() const {
    const auto now_steady = std::chrono::steady_clock::now();
    for (std::size_t index = 0; index < critical_feedback_seen_.size(); ++index) {
      if (!critical_feedback_seen_[index] ||
        std::chrono::duration<double>(now_steady - critical_feedback_time_[index]).count() >
        feedback_timeout_sec_)
      {
        return false;
      }
    }
    return true;
  }

  bool feedback_health_allows_tx(std::string & reason, bool for_arming = false) const {
    if (!critical_feedback_fresh()) {
      reason = "critical chassis feedback is missing or stale";
      return false;
    }
    const bool stop_state_allowed = VCUIDU_Fdbk_ModelStop == 0U ||
      (allow_arm_with_mechanical_estop_ && VCUIDU_Fdbk_ModelStop == 2U);
    if (!stop_state_allowed) {
      reason = "VCU stop state is not permitted";
      return false;
    }
    const bool vcu_handshake_grace = VCUIDU_Fdbk_Erro == 1U &&
      (for_arming || (armed_ && std::chrono::duration<double>(
      std::chrono::steady_clock::now() - armed_since_).count() <= vcu_handshake_grace_sec_));
    if ((VCUIDU_Fdbk_Erro != 0U && !vcu_handshake_grace) ||
      MCUIDU_Fdbk_ErroLevel != 0U ||
      MCUIDU_Fdbk_Fault != 0U || EPBIDU_Fdbk_ErroLevel != 0U ||
      EPBIDU_Fdbk_Fault != 0U || EPSIDU_Fdbk_ErroLevel != 0U ||
      EPSIDU_Fdbk_Fault != 0U || EHBIDU_Fdbk_ErroLevel != 0U ||
      EHBIDU_Fdbk_Fault != 0U)
    {
      reason = "one or more chassis controllers report a fault";
      return false;
    }
    return true;
  }

  bool feedback_state_allows_arm(std::string & reason) const {
    if (!feedback_health_allows_tx(reason, true)) {
      return false;
    }
    if (std::abs(MCUIDU_Fdbk_RealSped) > stationary_speed_raw_) {
      reason = "vehicle is not stationary";
      return false;
    }
    if (EPBIDU_Fdbk_ParkSta != 3U) {
      reason = "parking brake feedback is not applied";
      return false;
    }
    return true;
  }

  void handle_arm_request(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    if (!request->data) {
      armed_ = false;
      publish_armed_state();
      response->success = true;
      response->message = "CAN transmit path disarmed";
      return;
    }
    if (!enable_tx_) {
      response->success = false;
      response->message = "enable_tx is false; node is receive-only";
      return;
    }
    const bool command_fresh = have_command_ &&
      std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_command_).count() <= command_timeout_sec_;
    if (!command_fresh) {
      response->success = false;
      response->message = "no fresh bounded chassis command is available";
      return;
    }
    std::string reason;
    if (!feedback_state_allows_arm(reason)) {
      response->success = false;
      response->message = reason;
      return;
    }
    armed_ = true;
    armed_since_ = std::chrono::steady_clock::now();
    publish_armed_state();
    response->success = true;
    response->message = "CAN transmit path armed";
    RCLCPP_WARN(this->get_logger(), "CAN transmit path ARMED");
  }

  void data_callback() {
    if (!can_socket_->service()) {
      if (can_socket_->take_connection_reset()) {
        reset_feedback_state();
      }
      return;
    }
    if (can_socket_->take_connection_reset()) {
      reset_feedback_state();
    }
    nbytes = read(can_socket_->fd(), &frame, sizeof(frame)); //接收报文
    if (nbytes < 0) {
      if (errno != EAGAIN && errno != EWOULDBLOCK) {
        can_socket_->handle_io_error("SocketCAN read", errno);
        reset_feedback_state();
      }
      return;
    }
    if (nbytes != 0 && nbytes != static_cast<ssize_t>(sizeof(frame))) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Discarding partial SocketCAN frame (%zd bytes)", nbytes);
      return;
    }
    if (nbytes > 0 &&
      (frame.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG)) != 0U)
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Discarding non-standard-data SocketCAN frame 0x%08x", frame.can_id);
      return;
    }
    if (nbytes > 0 && frame.can_dlc != 8) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Discarding SocketCAN frame 0x%03x with DLC %u",
        frame.can_id & CAN_SFF_MASK, frame.can_dlc);
      return;
    }
    //报文解析
    if(nbytes > 0){
      const canid_t received_id = frame.can_id & CAN_SFF_MASK;
      const bool valid_feedback_id =
        received_id == 0x352 || received_id == 0x354 || received_id == 0x356 ||
        received_id == 0x358 || received_id == 0x360 || received_id == 0x362 ||
        received_id == 0x364;
      if(received_id == 0x352){
        /*校验和异或*/
        VCUIDU_Fdbk_Checksum = frame.data[7];
        check_sum_counter = frame.data[0]^frame.data[1]^frame.data[2]^frame.data[3]^frame.data[4]^frame.data[5]^frame.data[6];
        all_publisher_message.vcuidu_fdbk_checksum=VCUIDU_Fdbk_Checksum;
        if(check_sum_counter!=VCUIDU_Fdbk_Checksum){
          RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "VCUIDU(can_id 0x352) check sum ERROR!");
          return;
        }else{
          bitset<32> temp;
          bitset<8> bs(frame.data[0]);//转换二进制
          bitset<8> bs1(frame.data[1]);//转换二进制
          bitset<8> bs2(frame.data[2]);//转换二进制
          bitset<8> bs3(frame.data[3]);//转换二进制
          bitset<8> bs4(frame.data[4]);//转换二进制
          bitset<8> bs5(frame.data[5]);//转换二进制
          bitset<8> bs6(frame.data[6]);//转换二进制
          /*当前驾驶模式状态*/
         
          
          temp.reset();
          for(int i=0;i<2;i++) temp[i]=bs[i];
          VCUIDU_Fdbk_WorkMod=temp.to_ulong();
          all_publisher_message.vcuidu_fdbk_workmod=VCUIDU_Fdbk_WorkMod;
         
          // VCUIDU_Fdbk_WorkMod = frame.data[0]%0x04;
          // all_publisher_message.vcuidu_fdbk_workmod=VCUIDU_Fdbk_WorkMod;
          /*当前停止状态*/
          temp.reset();
          for(int i=0;i<3;i++) temp[i]=bs[i+2];
          VCUIDU_Fdbk_ModelStop = temp.to_ulong();//frame.data[0]%0x20/0x04;
          all_publisher_message.vcuidu_fdbk_modelstop=VCUIDU_Fdbk_ModelStop;
          /*当前点火状态*/
          temp.reset();
          for(int i=0;i<1;i++) temp[i]=bs[i+5];
          VCUIDU_Fdbk_Power =  temp.to_ulong();//frame.data[0]%0x40/0x20;
          all_publisher_message.vcuidu_fdbk_power=VCUIDU_Fdbk_Power;
          /*VCU故障信息*/
          temp.reset();
          for(int i=0;i<3;i++) temp[i]=bs1[i];
          VCUIDU_Fdbk_Erro = temp.to_ulong();//frame.data[1]%0x08;
          all_publisher_message.vcuidu_fdbk_erro=VCUIDU_Fdbk_Erro;
          /*VCU软件版本*/
          temp.reset();
          for(int i=0;i<8;i++) temp[i]=bs2[i];
          for(int i=0;i<8;i++) temp[i+8]=bs3[i];
          for(int i=0;i<8;i++) temp[i+16]=bs4[i];
          for(int i=0;i<4;i++) temp[i+24]=bs5[i];
          VCU_Fdbk_SoftVer = temp.to_ulong();//((frame.data[5]%0x10*256+frame.data[4])*256+frame.data[3])*256+frame.data[2];
          all_publisher_message.vcu_fdbk_softver=VCU_Fdbk_SoftVer;
          /*循环计数*/
          temp.reset();
          for(int i=0;i<4;i++) temp[i]=bs6[i];
          VCU_Fdbk_RollCnt = temp.to_ulong();//frame.data[6]%0x10;
          all_publisher_message.vcu_fdbk_rollcnt=VCU_Fdbk_RollCnt;
        }
        critical_feedback_seen_[0] = true;
        critical_feedback_time_[0] = std::chrono::steady_clock::now();
        can_socket_->mark_valid_feedback();
      }
      else if(received_id == 0x354){
        bitset<16> temp;
        bitset<8> bs(frame.data[0]);//转换二进制
        bitset<8> bs1(frame.data[1]);//转换二进制
        bitset<8> bs2(frame.data[2]);//转换二进制
        bitset<8> bs3(frame.data[3]);//转换二进制
        bitset<8> bs4(frame.data[4]);//转换二进制
        bitset<8> bs5(frame.data[5]);//转换二进制
        bitset<8> bs6(frame.data[6]);//转换二进制
        bitset<8> bs7(frame.data[7]);//转换二进制
        /*当前档位*/
        temp.reset();
        for(int i=0;i<2;i++) temp[i]=bs[i];
        MCUIDU_Fdbk_RND = temp.to_ulong();//frame.data[0]%0x04;
        all_publisher_message.mcuidu_fdbk_rnd=MCUIDU_Fdbk_RND;
        /*车辆实际速度*/
        temp.reset();
        for(int i=0;i<6;i++) temp[i]=bs[i+2];
        for(int i=0;i<6;i++) temp[i+6]=bs1[i];
        MCUIDU_Fdbk_RealSped = (int)temp.to_ulong();

        if(bs1.test(5)){
        MCUIDU_Fdbk_RealSped = MCUIDU_Fdbk_RealSped-0x1000;
        // RCLCPP_INFO(this->get_logger(),"MCUIDU_Fdbk_RealSped real data : %d",MCUIDU_Fdbk_RealSped);
        //RCLCPP_INFO(this->get_logger(),"MCUIDU_Fdbk_RealSped real data : %d",MCUIDU_Fdbk_RealSped);
        }
        //std::cout<<MCUIDU_Fdbk_RealSped<<std::endl;
        //else{
        //   MCUIDU_Fdbk_RealSped = (frame.data[1]%0x40*0x100+frame.data[0])/0x04;
        // }
        all_publisher_message.mcuidu_fdbk_realsped=MCUIDU_Fdbk_RealSped;
        /*发布车辆运行速度信息*/
        speed_publisher_float = MCUIDU_Fdbk_RealSped;
        speed_publisher_message.data = speed_publisher_float;
        speed_publisher_->publish(speed_publisher_message);
        // RCLCPP_INFO(this->get_logger(),"MCUIDU_Fdbk_RealSped raw data : %x %x, inter data : %d , real data : %f",frame.data[1],frame.data[0],MCUIDU_Fdbk_RealSped,speed_publisher_float);
        
        /*车辆实际加速度*/
        temp.reset();
        for(int i=0;i<2;i++) temp[i]=bs1[i+6];
        for(int i=0;i<8;i++) temp[i+2]=bs2[i];
        for(int i=0;i<2;i++) temp[i+10]=bs3[i];
        MCUIDU_Fdbk_RealAcc = (int)temp.to_ulong();
        if(bs3.test(1)){//负数情况
        MCUIDU_Fdbk_RealAcc = MCUIDU_Fdbk_RealAcc-0x1000;
        }
        all_publisher_message.mcuidu_fdbk_realacc=MCUIDU_Fdbk_RealAcc;
        /*实际车辆输出扭矩*/
        temp.reset();
        for(int i=0;i<6;i++) temp[i]=bs3[i+2];
        for(int i=0;i<6;i++) temp[i+6]=bs4[i];
        MCUIDU_Fdbk_RealTorq = temp.to_ulong()* torque_coefficient;//((frame.data[3]%0x40*0x100+frame.data[4])/0x04) * torque_coefficient;
        all_publisher_message.mcuidu_fdbk_realtorq=MCUIDU_Fdbk_RealTorq;
        /*电机转速*/
        temp.reset();
        for(int i=0;i<2;i++) temp[i]=bs4[i+6];
        for(int i=0;i<8;i++) temp[i+2]=bs5[i];
        for(int i=0;i<6;i++) temp[i+10]=bs6[i];
        MCUIDU_Fdbk_RealRpm =  temp.to_ulong();
        all_publisher_message.mcuidu_fdbk_realrpm=MCUIDU_Fdbk_RealRpm;
        // RCLCPP_INFO(this->get_logger(),"MCUIDU_Fdbk_RealRpm real data : %d",MCUIDU_Fdbk_RealRpm);

        /*故障等级*/
        temp.reset();
        for(int i=0;i<2;i++) temp[i]=bs6[i+6];
        MCUIDU_Fdbk_ErroLevel =  temp.to_ulong();
        all_publisher_message.mcuidu_fdbk_errolevel=MCUIDU_Fdbk_ErroLevel;
        /*故障信息*/
        // MCUIDU_Fdbk_Fault = frame.data[7];
        temp.reset();
        for(int i=0;i<8;i++) temp[i]=bs7[i];
        MCUIDU_Fdbk_Fault =  temp.to_ulong();
        all_publisher_message.mcuidu_fdbk_fault=MCUIDU_Fdbk_Fault;
        critical_feedback_seen_[1] = true;
        critical_feedback_time_[1] = std::chrono::steady_clock::now();
        can_socket_->mark_valid_feedback();
      }
      else if(received_id == 0x356){
        bitset<16> temp;
        bitset<8> bs(frame.data[0]);//转换二进制
        bitset<8> bs1(frame.data[1]);//转换二进制
        bitset<8> bs2(frame.data[2]);//转换二进制
        bitset<8> bs3(frame.data[3]);//转换二进制
        bitset<8> bs4(frame.data[4]);//转换二进制
        bitset<8> bs5(frame.data[5]);//转换二进制
        bitset<8> bs6(frame.data[6]);//转换二进制
        bitset<8> bs7(frame.data[7]);//转换二进制
        /*驻车状态*/
        temp.reset();
        for(int i=0;i<3;i++) temp[i]=bs[i];
        EPBIDU_Fdbk_ParkSta =  temp.to_ulong();
        all_publisher_message.epbidu_fdbk_parksta=EPBIDU_Fdbk_ParkSta;
        /*驻车系统的故障等级*/
        temp.reset();
        for(int i=0;i<8;i++) temp[i]=bs1[i];
        EPBIDU_Fdbk_ErroLevel =  temp.to_ulong();
        // EPBIDU_Fdbk_ErroLevel = frame.data[1];
        all_publisher_message.epbidu_fdbk_errolevel=EPBIDU_Fdbk_ErroLevel;
        /*驻车故障状态*/
        temp.reset();
        for(int i=0;i<8;i++) temp[i]=bs2[i];
        EPBIDU_Fdbk_Fault =  temp.to_ulong();
        // EPBIDU_Fdbk_Fault = frame.data[2];
        all_publisher_message.epbidu_fdbk_fault=EPBIDU_Fdbk_Fault;
        critical_feedback_seen_[2] = true;
        critical_feedback_time_[2] = std::chrono::steady_clock::now();
        can_socket_->mark_valid_feedback();
      }
      else if(received_id == 0x358){
        bitset<16> temp;
        bitset<8> bs(frame.data[0]);//转换二进制
        bitset<8> bs1(frame.data[1]);//转换二进制
        bitset<8> bs2(frame.data[2]);//转换二进制
        bitset<8> bs3(frame.data[3]);//转换二进制
        bitset<8> bs4(frame.data[4]);//转换二进制
        bitset<8> bs5(frame.data[5]);//转换二进制
        bitset<8> bs6(frame.data[6]);//转换二进制
        bitset<8> bs7(frame.data[7]);//转换二进制
        /*前轮实际转角*/
        temp.reset();
        for(int i=0;i<8;i++) temp[i]=bs[i];
        for(int i=0;i<8;i++) temp[i+8]=bs1[i];
        EPSIDU_Fdbk_RealAngle = (int)temp.to_ulong();
        if(bs1.test(3)){//12-bit signed value ends at byte 1 bit 3
          EPSIDU_Fdbk_RealAngle = EPSIDU_Fdbk_RealAngle-0x10000;//运算得到负数
          //RCLCPP_INFO(this->get_logger(),"EPSIDU_Fdbk_RealAngle real data : %d",EPSIDU_Fdbk_RealAngle);
        }
        all_publisher_message.epsidu_fdbk_realangle=EPSIDU_Fdbk_RealAngle;
        // RCLCPP_INFO(this->get_logger(),"EPSIDU_Fdbk_RealAngle raw data : %x %x",frame.data[1],frame.data[0]);
        /*发布前轮转向角度信息*/
        steerAngle_publisher_float = EPSIDU_Fdbk_RealAngle*0.1;
        steerAngle_publisher_message.data = steerAngle_publisher_float;
        steerAngle_publisher_->publish(steerAngle_publisher_message);

        /*工作状态*/
        temp.reset();
        for(int i=0;i<4;i++) temp[i]=bs2[i];
        EPSIDU_Fdbk_WorkSta = temp.to_ulong();
        all_publisher_message.epsidu_fdbk_worksta=EPSIDU_Fdbk_WorkSta;
        /*故障等级*/
        temp.reset();
        for(int i=0;i<4;i++) temp[i]=bs2[i+4];
        EPSIDU_Fdbk_ErroLevel = temp.to_ulong();
        all_publisher_message.epsidu_fdbk_errolevel=EPSIDU_Fdbk_ErroLevel;
        /*故障信息*/
        temp.reset();
        for(int i=0;i<8;i++) temp[i]=bs3[i];
        EPSIDU_Fdbk_Fault = temp.to_ulong();
        all_publisher_message.epsidu_fdbk_fault=EPSIDU_Fdbk_Fault;
        critical_feedback_seen_[3] = true;
        critical_feedback_time_[3] = std::chrono::steady_clock::now();
        can_socket_->mark_valid_feedback();
      }
      else if(received_id == 0x360){
        bitset<16> temp;
        bitset<8> bs(frame.data[0]);//转换二进制
        bitset<8> bs1(frame.data[1]);//转换二进制
        bitset<8> bs2(frame.data[2]);//转换二进制
        bitset<8> bs3(frame.data[3]);//转换二进制
        bitset<8> bs4(frame.data[4]);//转换二进制
        bitset<8> bs5(frame.data[5]);//转换二进制
        bitset<8> bs6(frame.data[6]);//转换二进制
        bitset<8> bs7(frame.data[7]);//转换二进制
        /*车辆实际减速度*/
        temp.reset();
        for(int i=0;i<8;i++) temp[i]=bs[i];
        for(int i=0;i<4;i++) temp[i+8]=bs1[i];
        EHBIDU_Fdbk_RealDece =  (int)temp.to_ulong();
        if(bs1.test(7)){//判断符号位 若为负数
          EHBIDU_Fdbk_RealDece = EHBIDU_Fdbk_RealDece-0x1000;//运算得到负数
        }
        all_publisher_message.ehbidu_fdbk_realdece =EHBIDU_Fdbk_RealDece ;
        /*实际制动油压值*/
        temp.reset();
        for(int i=0;i<8;i++) temp[i]=bs2[i];
        EHBIDU_Fdbk_RealPressure = temp.to_ulong();
        // EHBIDU_Fdbk_RealPressure = frame.data[2];
        all_publisher_message.ehbidu_fdbk_realpressure=EHBIDU_Fdbk_RealPressure;
        brake_publisher_message.data = EHBIDU_Fdbk_RealPressure;
        brake_publisher_->publish(brake_publisher_message);
        /*故障等级*/
        temp.reset();
        for(int i=0;i<8;i++) temp[i]=bs3[i];
        EHBIDU_Fdbk_ErroLevel = temp.to_ulong();
        // EHBIDU_Fdbk_ErroLevel = frame.data[3];
        all_publisher_message.ehbidu_fdbk_errolevel=EHBIDU_Fdbk_ErroLevel;
        /*故障信息*/
        temp.reset();
        for(int i=0;i<8;i++) temp[i]=bs4[i];
        EHBIDU_Fdbk_Fault = temp.to_ulong();
        // EHBIDU_Fdbk_Fault = frame.data[4];
        all_publisher_message.ehbidu_fdbk_fault=EHBIDU_Fdbk_Fault;
        critical_feedback_seen_[4] = true;
        critical_feedback_time_[4] = std::chrono::steady_clock::now();
        can_socket_->mark_valid_feedback();
      }
      else if(received_id == 0x362){
        bitset<16> temp;
        bitset<8> bs(frame.data[0]);//转换二进制
        bitset<8> bs1(frame.data[1]);//转换二进制
        bitset<8> bs2(frame.data[2]);//转换二进制
        bitset<8> bs3(frame.data[3]);//转换二进制
        bitset<8> bs4(frame.data[4]);//转换二进制
        bitset<8> bs5(frame.data[5]);//转换二进制
        bitset<8> bs6(frame.data[6]);//转换二进制
        bitset<8> bs7(frame.data[7]);//转换二进制
        /*充电状态*/
        temp.reset();
        for(int i=0;i<1;i++) temp[i]=bs[i];
        BMSIDU_Fdbk_CHCSta = temp.to_ulong();
        // BMSIDU_Fdbk_CHCSta = frame.data[0]%0x02;
        all_publisher_message.bmsidu_fdbk_chcsta=BMSIDU_Fdbk_CHCSta;
        /*剩余电量(SOC)*/
        temp.reset();
        for(int i=0;i<7;i++) temp[i]=bs[i+1];
        for(int i=0;i<5;i++) temp[i+7]=bs1[i];
        BMSIDU_Fdbk_SOC = temp.to_ulong();
        // BMSIDU_Fdbk_SOC = (frame.data[1]%0x20*0x100+frame.data[0])/0x02;
        all_publisher_message.bmsidu_fdbk_soc=BMSIDU_Fdbk_SOC;
        soc_publisher_message.data=BMSIDU_Fdbk_SOC;
        soc_publisher_->publish(soc_publisher_message);
        /*母线电流*/
        temp.reset();
        for(int i=0;i<8;i++) temp[i]=bs2[i];
        for(int i=0;i<8;i++) temp[i+8]=bs3[i];
        BMSIDU_Fdbk_BusA = (int)temp.to_ulong();
        if(bs3.test(7)){
          BMSIDU_Fdbk_BusA = BMSIDU_Fdbk_BusA - 0x10000;
        }
        all_publisher_message.bmsidu_fdbk_busa=BMSIDU_Fdbk_BusA;
        /*母线电压*/
        temp.reset();
        for(int i=0;i<8;i++) temp[i]=bs4[i];
        for(int i=0;i<8;i++) temp[i+8]=bs5[i];
        BMSIDU_Fdbk_BusV = temp.to_ulong();
        all_publisher_message.bmsidu_fdbk_busv=BMSIDU_Fdbk_BusV;
        /*故障等级*/
        temp.reset();
        for(int i=0;i<3;i++) temp[i]=bs1[i+5];
        BMSIDU_Fdbk_ErroLevel = temp.to_ulong();
        all_publisher_message.bmsidu_fdbk_errolevel=BMSIDU_Fdbk_ErroLevel;
        /*故障信息*/
        temp.reset();
        for(int i=0;i<8;i++) temp[i]=bs6[i];
        BMSIDU_Fdbk_Fault = temp.to_ulong();
        all_publisher_message.bmsidu_fdbk_fault=BMSIDU_Fdbk_Fault;
        can_socket_->mark_valid_feedback();
      }
      else if(received_id == 0x364){
        bitset<16> temp;
        bitset<8> bs(frame.data[0]);//转换二进制
        bitset<8> bs1(frame.data[1]);//转换二进制
        bitset<8> bs2(frame.data[2]);//转换二进制
        bitset<8> bs3(frame.data[3]);//转换二进制
        bitset<8> bs4(frame.data[4]);//转换二进制
        bitset<8> bs5(frame.data[5]);//转换二进制
        bitset<8> bs6(frame.data[6]);//转换二进制
        bitset<8> bs7(frame.data[7]);//转换二进制
        /*大灯状态反馈*/
        temp.reset();
        for(int i=0;i<2;i++) temp[i]=bs[i];
        BCMIDU_Fdbk_HeadLight = temp.to_ulong();
        all_publisher_message.bcmidu_fdbk_headlight=BCMIDU_Fdbk_HeadLight;
        /*转向状态反馈*/
        temp.reset();
        for(int i=0;i<2;i++) temp[i]=bs[i+2];
        BCMIDU_Fdbk_TurnFlash = temp.to_ulong();
        all_publisher_message.bcmidu_fdbk_turnflash=BCMIDU_Fdbk_TurnFlash;
        /*倒车灯状态反馈*/
        temp.reset();
        for(int i=0;i<2;i++) temp[i]=bs[i+4];
        BCMIDU_Fdbk_BackLight = temp.to_ulong();
        all_publisher_message.bcmidu_fdbk_backlight=BCMIDU_Fdbk_BackLight;
        /*刹车灯状态反馈*/
        temp.reset();
        for(int i=0;i<2;i++) temp[i]=bs[i+6];
        BCMIDU_Fdbk_BrakeLight = temp.to_ulong();
        all_publisher_message.bcmidu_fdbk_brakelight=BCMIDU_Fdbk_BrakeLight;
        /*喇叭控制*/
        temp.reset();
        for(int i=0;i<2;i++) temp[i]=bs1[i];
        BCMIDU_Fdbk_Siren = temp.to_ulong();
        all_publisher_message.bcmidu_fdbk_siren=BCMIDU_Fdbk_Siren;
        /*行车对外语音播报*/
        temp.reset();
        for(int i=0;i<2;i++) temp[i]=bs1[i+2];
        BCMIDU_Fdbk_Voice = temp.to_ulong();
        all_publisher_message.bcmidu_fdbk_voice=BCMIDU_Fdbk_Voice;
        /*双闪反馈*/
        temp.reset();
        for(int i=0;i<2;i++) temp[i]=bs1[i+4];
        BCMIDU_Fdbk_DoubleFlash = temp.to_ulong();
        all_publisher_message.bcmidu_fdbk_doubleflash=BCMIDU_Fdbk_DoubleFlash;
        can_socket_->mark_valid_feedback();
      }

      if (!valid_feedback_id) {
        return;
      }

      all_publisher_message.header.stamp = now();
      all_publisher_message.header.frame_id = CAN_name;
      all_publisher_->publish(all_publisher_message);

      std::stringstream ss;
      ss << VCUIDU_Fdbk_WorkMod << ","  << VCUIDU_Fdbk_ModelStop << ","  << VCUIDU_Fdbk_Power << ","  << VCUIDU_Fdbk_Erro << ","  << VCU_Fdbk_SoftVer << ","  << VCU_Fdbk_RollCnt << ","  << VCUIDU_Fdbk_Checksum << ";"  << MCUIDU_Fdbk_RND << ","  << MCUIDU_Fdbk_RealSped << ","  << MCUIDU_Fdbk_RealAcc << ","  << MCUIDU_Fdbk_RealTorq << ","  << MCUIDU_Fdbk_RealRpm << ","  << MCUIDU_Fdbk_ErroLevel << ","  << MCUIDU_Fdbk_Fault << ";"  << EPBIDU_Fdbk_ParkSta << ","  << EPBIDU_Fdbk_ErroLevel << ","  << EPBIDU_Fdbk_Fault << ";"  << EPSIDU_Fdbk_RealAngle << ","  << EPSIDU_Fdbk_WorkSta << ","  << EPSIDU_Fdbk_ErroLevel << ","  << EPSIDU_Fdbk_Fault << ";"  << EHBIDU_Fdbk_RealDece << ","  << EHBIDU_Fdbk_RealPressure << ","  << EHBIDU_Fdbk_ErroLevel << ","  << EHBIDU_Fdbk_Fault << ";"  << BMSIDU_Fdbk_CHCSta << ","  << BMSIDU_Fdbk_SOC << ","  << BMSIDU_Fdbk_BusA << ","  << BMSIDU_Fdbk_BusV << ","  << BMSIDU_Fdbk_ErroLevel << ","  << BMSIDU_Fdbk_Fault << ";"  << BCMIDU_Fdbk_HeadLight << ","  << BCMIDU_Fdbk_TurnFlash << ","  << BCMIDU_Fdbk_BackLight << ","  << BCMIDU_Fdbk_BrakeLight << ","  << BCMIDU_Fdbk_Siren << ","  << BCMIDU_Fdbk_Voice << ","  << BCMIDU_Fdbk_DoubleFlash ;
      ss >> rawData_publisher_message.data;
      rawData_publisher_->publish(rawData_publisher_message);
    }
  }

  void reset_feedback_state()
  {
    critical_feedback_seen_.fill(false);
    disarm("SocketCAN connection was lost or reset");
  }

  /*订阅控制速度指令回调函数*/
  void command_callback_chassis_danger(const std_msgs::msg::Int16::SharedPtr msg_chassis_danger){
    danger_flag = msg_chassis_danger->data;
  }

  // /*订阅控制速度指令回调函数*/
  // void command_callback_chassis_speed(const std_msgs::msg::Int16::SharedPtr msg_chassis_speed){
  //   //if(have_rev_controlCmd) return;
  //   IDUMCU_Ctrl_TgtSped = msg_chassis_speed->data;
  //   if(IDUMCU_Ctrl_TgtSped>0.1){
  //     IDUMCU_Ctrl_RND = 0x1; //挂前进档D
  //   }else if(IDUMCU_Ctrl_TgtSped<-0.1){
  //     IDUMCU_Ctrl_RND = 0x2; //挂倒车档R
  //   }else{
  //     IDUMCU_Ctrl_RND = 0x0; //挂空档N
  //   }
  //   //have_rev_controlCmd=true;
  // }

  // /*订阅控制转向角度指令回调函数*/
  // void command_callback_chassis_steerAngle(const std_msgs::msg::Float32::SharedPtr msg_chassis_steerAngle){
  //   //if(have_rev_controlCmd) return;
  //   IDUEPS_Ctrl_TgtAngle = int(msg_chassis_steerAngle->data*10*lr_signal);
  //   // RCLCPP_INFO(this->get_logger(),"the control target angle is %d",IDUEPS_Ctrl_TgtAngle);
  //   //have_rev_controlCmd=true;
  // }

  // /*订阅控制制动压力指令回调函数*/
  // void command_callback_chassis_brake(const std_msgs::msg::Float32::SharedPtr msg_chassis_brake){
  //   //if(have_rev_controlCmd) return;
  //   IDUEHB_Ctrl_BrakMod = 0x2; //控制制动油压模式
  //   IDUEHB_Ctrl_TgtPressure = int(msg_chassis_brake->data);
  //   //have_rev_controlCmd=true;
  // }

  /**/
  void command_callback_chassis_can_sended(const sensor_driver_msgs::msg::ChassisCanSended::SharedPtr msg){
    IDUVCU_Ctrl_WorkMod=msg->iduvcu_ctrl_workmod;
    IDUVCU_Ctrl_ESTOP=msg->iduvcu_ctrl_estop;
    IDUVCU_Ctrl_ReStop=msg->iduvcu_ctrl_restop;
    IDUMCU_Ctrl_WorkMod=msg->idumcu_ctrl_workmod;
    IDUMCU_Ctrl_RND=msg->idumcu_ctrl_rnd;
    IDUMCU_Ctrl_TgtSped=msg->idumcu_ctrl_tgtsped;
    IDUMCU_Ctrl_TgtAcc=msg->idumcu_ctrl_tgtacc;
    IDUMCU_Ctrl_TgtTorq=msg->idumcu_ctrl_tgttorq;
    IDUEPB_Ctrl_ReqPark=msg->iduepb_ctrl_reqpark;
    IDUEPS_Ctrl_TgtAngle=msg->idueps_ctrl_tgtangle;
    IDUEHB_Ctrl_BrakMod=msg->iduehb_ctrl_brakmod;
    IDUEHB_Ctrl_TgtPressure=msg->iduehb_ctrl_tgtpressure;
    IDUEHB_Ctrl_TgtDece=msg->iduehb_ctrl_tgtdece;
    IDUBMS_Ctrl_Power=msg->idubms_ctrl_power;
    IDUBCM_Ctrl_HeadLight=msg->idubcm_ctrl_headlight;
    IDUBCM_Ctrl_LeftFlash=msg->idubcm_ctrl_leftflash;
    IDUBCM_Ctrl_RightFlash=msg->idubcm_ctrl_rightflash;
    IDUBCM_Ctrl_BackLight=msg->idubcm_ctrl_backlight;
    IDUBCM_Ctrl_BrakeLight=msg->idubcm_ctrl_brakelight;
    IDUBCM_Ctrl_Siren=msg->idubcm_ctrl_siren;
    IDUBCM_Ctrl_Voice=msg->idubcm_ctrl_voice;
    IDUBCM_Ctrl_DoubleFlash=msg->idubcm_ctrl_doubleflash;
    last_command_ = std::chrono::steady_clock::now();
    have_command_ = true;
    
    /*according to the danger state, do some emergency action*/
    // if(danger_flag==1)
    // {
    // 	IDUBCM_Ctrl_DoubleFlash=1;
    // 	IDUMCU_Ctrl_TgtSped=0;
    // }	
    // else if(danger_flag==2)
    // {
    // 	IDUBCM_Ctrl_DoubleFlash=1;
    // 	IDUBCM_Ctrl_BackLight=1;
    // 	IDUMCU_Ctrl_TgtSped=0;
    // 	IDUEHB_Ctrl_TgtPressure=5;
    // }
    // else
    // {
    // 	IDUBCM_Ctrl_DoubleFlash=0;
    // }
  }

  /*
    *将目标车辆状态变量转换为底盘can控制信号
    *并且通过串口发送给车辆底盘
  */
  void data_sendCAN() {
    // TX is never allowed to create or reconnect a socket.  The receive timer
    // owns connection health; with enable_tx=false no write path is scheduled.
    if (!enable_tx_ || !armed_ || !can_socket_->is_connected()) {
      return;
    }
    std::string feedback_reason;
    if (!feedback_health_allows_tx(feedback_reason)) {
      disarm(feedback_reason.c_str());
      return;
    }
    const bool command_fresh = have_command_ &&
      std::chrono::duration<double>(std::chrono::steady_clock::now() - last_command_).count() <=
      command_timeout_sec_;
    if (!command_fresh) {
      IDUVCU_Ctrl_WorkMod = 1U;
      IDUMCU_Ctrl_RND = MCUIDU_Fdbk_RND;
      IDUMCU_Ctrl_TgtSped = 0;
      IDUMCU_Ctrl_TgtAcc = 0;
      IDUMCU_Ctrl_TgtTorq = 0;
      IDUEPB_Ctrl_ReqPark =
        std::abs(MCUIDU_Fdbk_RealSped) <= stationary_speed_raw_ ? 1U : 2U;
      IDUEHB_Ctrl_BrakMod = 2U;
      IDUEHB_Ctrl_TgtPressure = 10U;
      IDUEHB_Ctrl_TgtDece = 0;
      IDUBCM_Ctrl_BrakeLight = 1U;
      IDUBCM_Ctrl_DoubleFlash = 1U;
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "No fresh chassis command; transmitting fail-safe stop");
    }
    IDUMCU_Ctrl_TgtSped = std::clamp(IDUMCU_Ctrl_TgtSped, -2048, 2047);
    IDUMCU_Ctrl_TgtAcc = std::clamp(IDUMCU_Ctrl_TgtAcc, -2048, 2047);
    IDUMCU_Ctrl_TgtTorq = std::clamp(IDUMCU_Ctrl_TgtTorq, -2048, 2047);
    IDUEHB_Ctrl_TgtDece = std::clamp(IDUEHB_Ctrl_TgtDece, -2048, 2047);
    IDUEHB_Ctrl_TgtPressure = std::min(IDUEHB_Ctrl_TgtPressure, 255U);
    //设置bitset对象辅助进行位运算    
    bitset<32> low; //can消息的低4字节
    bitset<32> high; //can消息的高4字节
    unsigned long int low_t=0; //bitset对应
    unsigned long int high_t=0;
    if(IDUVCU_Ctrl_RollCnt<15){
      IDUVCU_Ctrl_RollCnt = IDUVCU_Ctrl_RollCnt + 1;
    }else{
      IDUVCU_Ctrl_RollCnt = 0;
    }

    //地址0x351，VCU_Ctrl
    low.reset(); //复位
    high.reset();
    if(IDUVCU_Ctrl_WorkMod==1) low.set(0);
    if(IDUVCU_Ctrl_ESTOP==1) low.set(1);
    if(IDUVCU_Ctrl_ReStop==1) low.set(2);
    bitset<4>temp0(IDUVCU_Ctrl_RollCnt);
    for(int i=0;i<4;i++) high[i+16]=temp0[i];
    low_t=low.to_ulong();
    high_t=high.to_ulong();
    for(int i=0;i<4;i++) can_vec[0].data[i]=((char*)(&low_t))[i]; //指针类型强制转换，按照每字节进行寻址
    for(int i=0;i<4;i++) can_vec[0].data[i+4]=((char*)(&high_t))[i];
    IDUVCU_Ctrl_Checksum = can_vec[0].data[0]^can_vec[0].data[1]^can_vec[0].data[2]^can_vec[0].data[3]^can_vec[0].data[4]^can_vec[0].data[5]^can_vec[0].data[6];
    bitset<8>temp1(IDUVCU_Ctrl_Checksum);
    for(int i=0;i<8;i++) high[i+24]=temp1[i];
    low_t=low.to_ulong();
    high_t=high.to_ulong();
    for(int i=0;i<4;i++) can_vec[0].data[i]=((char*)(&low_t))[i];
    for(int i=0;i<4;i++) can_vec[0].data[i+4]=((char*)(&high_t))[i];

    //地址0x353,MCU_Ctrl
    low.reset();
    high.reset();
    bitset<3>temp2(IDUMCU_Ctrl_WorkMod);
    for(int i=0;i<3;i++) low[i+1]=temp2[i];
    bitset<2>temp3(IDUMCU_Ctrl_RND);
    for(int i=0;i<2;i++) low[i+4]=temp3[i];
    bitset<12>temp4(IDUMCU_Ctrl_TgtSped);
    for(int i=0;i<2;i++) low[i+6]=temp4[i];
    for(int i=0;i<8;i++) low[i+8]=temp4[i+2];
    for(int i=0;i<2;i++) low[i+16]=temp4[i+10];
    bitset<12>temp5(IDUMCU_Ctrl_TgtAcc);
    for(int i=0;i<12;i++) low[i+18]=temp5[i];
    bitset<12>temp6(IDUMCU_Ctrl_TgtTorq);
    for(int i=0;i<12;i++) high[i+10]=temp6[i];
    low_t=low.to_ulong();
    high_t=high.to_ulong();
    for(int i=0;i<4;i++) can_vec[1].data[i]=((char*)(&low_t))[i];
    for(int i=0;i<4;i++) can_vec[1].data[i+4]=((char*)(&high_t))[i];

    //地址0x355,EPB_Ctrl
    low.reset();
    high.reset();
    bitset<2>temp7(IDUEPB_Ctrl_ReqPark);
    for(int i=0;i<2;i++) low[i]=temp7[i];
    low_t=low.to_ulong();
    can_vec[2].data[0]=((char*)(&low_t))[0];

    //地址0x357,EPS_Ctrl
    low.reset();
    high.reset();
    bitset<16>temp8(IDUEPS_Ctrl_TgtAngle);
    for(int i=0;i<16;i++) low[i]=temp8[i];
    bitset<8>temp_angelSped(150);
    for(int i=0;i<8;i++) low[i+16]=temp_angelSped[i];
    low_t=low.to_ulong();
    for(int i=0;i<3;i++) can_vec[3].data[i]=((char*)(&low_t))[i];
    
    //地址0x359,EHB_Ctrl
    low.reset();
    high.reset();
    bitset<2>temp9(IDUEHB_Ctrl_BrakMod);
    for(int i=0;i<2;i++) low[i]=temp9[i];
    bitset<8>temp10(IDUEHB_Ctrl_TgtPressure);
    for(int i=0;i<8;i++) low[i+2]=temp10[i];
    bitset<12>temp11(IDUEHB_Ctrl_TgtDece);
    for(int i=0;i<12;i++) low[i+16]=temp11[i];
    low_t=low.to_ulong();
    for(int i=0;i<4;i++) can_vec[4].data[i]=((char*)(&low_t))[i];

    //地址0x361,BMS_Ctrl
    low.reset();
    high.reset();
    if(IDUBMS_Ctrl_Power==1) low.set(0);
    low_t=low.to_ulong();
    can_vec[5].data[0]=((char*)(&low_t))[0];

    //地址0x363,BCM_Ctrl
    low.reset();
    high.reset();
    bitset<2>temp12(IDUBCM_Ctrl_HeadLight);
    for(int i=0;i<2;i++) low[i]=temp12[i];
    bitset<2>temp13(IDUBCM_Ctrl_LeftFlash);
    for(int i=0;i<2;i++) low[i+2]=temp13[i];
    bitset<2>temp14(IDUBCM_Ctrl_RightFlash);
    for(int i=0;i<2;i++) low[i+4]=temp14[i];
    bitset<2>temp15(IDUBCM_Ctrl_BackLight);
    for(int i=0;i<2;i++) low[i+6]=temp15[i];
    bitset<2>temp16(IDUBCM_Ctrl_BrakeLight);
    for(int i=0;i<2;i++) low[i+8]=temp16[i];
    bitset<2>temp17(IDUBCM_Ctrl_Siren);
    for(int i=0;i<2;i++) low[i+10]=temp17[i];
    bitset<2>temp18(IDUBCM_Ctrl_Voice);
    for(int i=0;i<2;i++) low[i+12]=temp18[i];
    bitset<2>temp19(IDUBCM_Ctrl_DoubleFlash);
    for(int i=0;i<2;i++) low[i+14]=temp19[i];
    low_t=low.to_ulong();
    for(int i=0;i<4;i++) can_vec[6].data[i]=((char*)(&low_t))[i];
  
    //向车辆发送can报文
    frame.can_dlc = 8;
    for(int i = 0; i < 7; i ++){
      frame.can_id = can_vec[i].can_id;
      for(int j = 0; j < 8; j ++) frame.data[j] = can_vec[i].data[j];
      nbytes = write(can_socket_->fd(), &frame, sizeof(frame)); //发送报文
      if (nbytes != static_cast<ssize_t>(sizeof(frame))) {
        if (nbytes < 0) {
          can_socket_->handle_io_error("SocketCAN write", errno);
          reset_feedback_state();
          break;
        } else {
          RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "SocketCAN partial write: %zd of %zu bytes", nbytes, sizeof(frame));
        }
      }
    }
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  /*创建对应节点的共享指针对象*/
  auto node = std::make_shared<chassis_can>("chassis_can_driver_node");
  /* 运行节点，并检测退出信号*/
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
