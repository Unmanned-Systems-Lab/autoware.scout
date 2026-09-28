#include "ez21_vehicle_launch/ez21_vehicle_interface_node.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace ez21_vehicle_launch
{
namespace
{

using drivers::socketcan::CanId;
using drivers::socketcan::ExtendedFrame;
using drivers::socketcan::FrameType;
using drivers::socketcan::SocketCanTimeout;
using drivers::socketcan::StandardFrame;

constexpr std::size_t kCanDataLength = 8U;
constexpr double kPi = 3.14159265358979323846;
constexpr double kDefaultMaxSteerAngleRad = kPi / 6.0;  // 30 deg
constexpr uint32_t kVcuCommandId = 0x00000102U;
constexpr uint32_t kVcuFeedbackId = 0x00000203U;
constexpr uint32_t kSteeringCommandId = 0x00000118U;
constexpr uint32_t kSteeringFaultFeedbackId = 0x00000080U;
constexpr uint32_t kSteeringFeedbackDirectId = 0x00000220U;
constexpr uint32_t kSteeringFeedbackRemoteId = 0x00000201U;
constexpr std::array<uint32_t, 4> kDriveCommandIds = {
  0x00070300U, 0x00070400U, 0x00070600U, 0x00070700U};
constexpr std::array<uint32_t, 4> kDriveFeedbackDirectIds = {
  0x180F0100U, 0x18130100U, 0x181B0100U, 0x181F0100U};
constexpr std::array<uint32_t, 4> kDriveFeedbackRemoteIds = {
  0x00000180U, 0x00000181U, 0x0000018BU, 0x0000018FU};
constexpr uint8_t kSteeringAutonomousMode = 0x31U;
constexpr uint8_t kSteeringEpsMode = 0x34U;
constexpr uint8_t kDriveEnableValue = 0x75U;

uint16_t read_le_u16(const std::array<uint8_t, kCanDataLength> & data, const std::size_t offset)
{
  return static_cast<uint16_t>(
    static_cast<uint16_t>(data.at(offset)) |
    (static_cast<uint16_t>(data.at(offset + 1U)) << 8U));
}

void write_le_u16(
  std::array<uint8_t, kCanDataLength> & data, const std::size_t offset, const uint16_t value)
{
  data.at(offset) = static_cast<uint8_t>(value & 0xFFU);
  data.at(offset + 1U) = static_cast<uint8_t>((value >> 8U) & 0xFFU);
}

void write_be_u16(
  std::array<uint8_t, kCanDataLength> & data, const std::size_t offset, const uint16_t value)
{
  data.at(offset) = static_cast<uint8_t>((value >> 8U) & 0xFFU);
  data.at(offset + 1U) = static_cast<uint8_t>(value & 0xFFU);
}

double clamp_abs(const double value, const double limit)
{
  return std::clamp(value, -limit, limit);
}

bool is_autonomous_request(const uint8_t mode)
{
  return
    mode == autoware_vehicle_msgs::srv::ControlModeCommand::Request::AUTONOMOUS ||
    mode == autoware_vehicle_msgs::srv::ControlModeCommand::Request::AUTONOMOUS_STEER_ONLY ||
    mode == autoware_vehicle_msgs::srv::ControlModeCommand::Request::AUTONOMOUS_VELOCITY_ONLY;
}

uint8_t requested_mode_to_report(const uint8_t mode)
{
  if (is_autonomous_request(mode)) {
    return autoware_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS;
  }
  if (mode == autoware_vehicle_msgs::srv::ControlModeCommand::Request::MANUAL) {
    return autoware_vehicle_msgs::msg::ControlModeReport::MANUAL;
  }
  return autoware_vehicle_msgs::msg::ControlModeReport::DISENGAGED;
}

uint8_t steering_raw_mode_to_report(const uint8_t raw_mode, const bool fault)
{
  if (fault) {
    return autoware_vehicle_msgs::msg::ControlModeReport::NOT_READY;
  }

  switch (raw_mode & 0x0FU) {
    case 0x01U:
      return autoware_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS;
    case 0x04U:
    case 0x05U:
      return autoware_vehicle_msgs::msg::ControlModeReport::MANUAL;
    case 0x06U:
    case 0x07U:
      return autoware_vehicle_msgs::msg::ControlModeReport::NOT_READY;
    default:
      return autoware_vehicle_msgs::msg::ControlModeReport::DISENGAGED;
  }
}

uint8_t gear_command_to_report(const uint8_t command)
{
  switch (command) {
    case autoware_vehicle_msgs::msg::GearCommand::DRIVE:
    case autoware_vehicle_msgs::msg::GearCommand::DRIVE_2:
    case autoware_vehicle_msgs::msg::GearCommand::DRIVE_3:
    case autoware_vehicle_msgs::msg::GearCommand::DRIVE_4:
    case autoware_vehicle_msgs::msg::GearCommand::DRIVE_5:
    case autoware_vehicle_msgs::msg::GearCommand::DRIVE_6:
    case autoware_vehicle_msgs::msg::GearCommand::DRIVE_7:
    case autoware_vehicle_msgs::msg::GearCommand::DRIVE_8:
    case autoware_vehicle_msgs::msg::GearCommand::DRIVE_9:
    case autoware_vehicle_msgs::msg::GearCommand::DRIVE_10:
    case autoware_vehicle_msgs::msg::GearCommand::DRIVE_11:
    case autoware_vehicle_msgs::msg::GearCommand::DRIVE_12:
    case autoware_vehicle_msgs::msg::GearCommand::DRIVE_13:
    case autoware_vehicle_msgs::msg::GearCommand::DRIVE_14:
    case autoware_vehicle_msgs::msg::GearCommand::DRIVE_15:
    case autoware_vehicle_msgs::msg::GearCommand::DRIVE_16:
    case autoware_vehicle_msgs::msg::GearCommand::DRIVE_17:
    case autoware_vehicle_msgs::msg::GearCommand::DRIVE_18:
      return autoware_vehicle_msgs::msg::GearReport::DRIVE;
    case autoware_vehicle_msgs::msg::GearCommand::REVERSE:
    case autoware_vehicle_msgs::msg::GearCommand::REVERSE_2:
      return autoware_vehicle_msgs::msg::GearReport::REVERSE;
    case autoware_vehicle_msgs::msg::GearCommand::PARK:
      return autoware_vehicle_msgs::msg::GearReport::PARK;
    case autoware_vehicle_msgs::msg::GearCommand::NEUTRAL:
      return autoware_vehicle_msgs::msg::GearReport::NEUTRAL;
    case autoware_vehicle_msgs::msg::GearCommand::LOW:
    case autoware_vehicle_msgs::msg::GearCommand::LOW_2:
      return autoware_vehicle_msgs::msg::GearReport::LOW;
    default:
      return autoware_vehicle_msgs::msg::GearReport::NONE;
  }
}

bool is_reverse_gear_report(const uint8_t report)
{
  return
    report == autoware_vehicle_msgs::msg::GearReport::REVERSE ||
    report == autoware_vehicle_msgs::msg::GearReport::REVERSE_2;
}

bool is_drive_gear_report(const uint8_t report)
{
  return
    report >= autoware_vehicle_msgs::msg::GearReport::DRIVE &&
    report <= autoware_vehicle_msgs::msg::GearReport::DRIVE_18;
}

bool is_motion_enabled_gear(const uint8_t report)
{
  return is_drive_gear_report(report) || is_reverse_gear_report(report);
}

double rpm_to_mps(const double wheel_rpm, const double wheel_radius_m)
{
  return wheel_rpm * (2.0 * kPi * wheel_radius_m) / 60.0;
}

double mps_to_rpm(const double velocity_mps, const double wheel_radius_m)
{
  if (wheel_radius_m <= std::numeric_limits<double>::epsilon()) {
    return 0.0;
  }
  return velocity_mps * 60.0 / (2.0 * kPi * wheel_radius_m);
}

double raw_to_steering_angle_rad(
  const uint16_t raw_value, const double center_raw, const double counts_per_radian)
{
  if (counts_per_radian <= std::numeric_limits<double>::epsilon()) {
    return 0.0;
  }
  return (static_cast<double>(raw_value) - center_raw) / counts_per_radian;
}

double vehicle_to_autoware_steering_angle_rad(const double vehicle_angle_rad)
{
  // Vehicle-side steering uses left negative/right positive; Autoware uses left positive/right negative.
  return -vehicle_angle_rad;
}

double autoware_to_vehicle_steering_angle_rad(const double autoware_angle_rad)
{
  return -autoware_angle_rad;
}

uint16_t steering_angle_rad_to_raw(
  const double angle_rad, const double center_raw, const double counts_per_radian,
  const double max_steer_angle_rad)
{
  const double clamped_angle = clamp_abs(angle_rad, max_steer_angle_rad);
  const double raw_value = center_raw + clamped_angle * counts_per_radian;
  const double bounded_raw =
    std::clamp(raw_value, 0.0, static_cast<double>(std::numeric_limits<uint16_t>::max()));
  return static_cast<uint16_t>(std::lround(bounded_raw));
}

uint8_t steering_speed_deg_per_s_to_raw(
  const double speed_deg_per_s, const double min_deg_per_s, const double max_deg_per_s)
{
  const double clamped_speed = std::clamp(speed_deg_per_s, min_deg_per_s, max_deg_per_s);
  return static_cast<uint8_t>(std::lround(clamped_speed / 10.0));
}

double decode_drive_feedback_rpm(const uint8_t raw_value)
{
  return static_cast<double>(static_cast<int>(raw_value) - 0x80) * 5.0;
}

bool any_nonzero(const std::array<uint8_t, kCanDataLength> & data)
{
  return std::any_of(data.begin(), data.end(), [](const uint8_t value) { return value != 0U; });
}

uint8_t clamp_ratio_to_percent(const double numerator, const double denominator)
{
  if (denominator <= std::numeric_limits<double>::epsilon()) {
    return 0U;
  }

  return static_cast<uint8_t>(
    std::lround(std::clamp(numerator / denominator, 0.0, 1.0) * 100.0));
}

void validate_lookup_table(
  const std::vector<double> & x_points, const std::vector<double> & y_points,
  const std::string & x_name, const std::string & y_name)
{
  if (x_points.empty()) {
    throw std::invalid_argument(x_name + " must not be empty");
  }
  if (x_points.size() != y_points.size()) {
    throw std::invalid_argument(x_name + " and " + y_name + " must have the same length");
  }

  double previous_x = -std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index < x_points.size(); ++index) {
    const double x = x_points.at(index);
    const double y = y_points.at(index);

    if (!std::isfinite(x)) {
      throw std::invalid_argument(x_name + " must contain only finite values");
    }
    if (!std::isfinite(y)) {
      throw std::invalid_argument(y_name + " must contain only finite values");
    }
    if (x < 0.0) {
      throw std::invalid_argument(x_name + " must contain only non-negative values");
    }
    if (y < 0.0 || y > 1.0) {
      throw std::invalid_argument(y_name + " must stay within [0.0, 1.0]");
    }
    if (index > 0U && x < previous_x) {
      throw std::invalid_argument(x_name + " must be sorted in non-decreasing order");
    }
    previous_x = x;
  }
}

double interpolate_lookup_table(
  const std::vector<double> & x_points, const std::vector<double> & y_points, const double query)
{
  if (x_points.empty() || y_points.empty()) {
    return 0.0;
  }

  if (query <= x_points.front()) {
    return y_points.front();
  }
  if (query >= x_points.back()) {
    return y_points.back();
  }

  for (std::size_t index = 1; index < x_points.size(); ++index) {
    const double x1 = x_points.at(index);
    if (query > x1) {
      continue;
    }

    const double x0 = x_points.at(index - 1U);
    const double y0 = y_points.at(index - 1U);
    const double y1 = y_points.at(index);
    const double range = x1 - x0;
    if (range <= std::numeric_limits<double>::epsilon()) {
      return y1;
    }

    const double alpha = (query - x0) / range;
    return y0 + alpha * (y1 - y0);
  }

  return y_points.back();
}

}  // namespace

Ez21VehicleInterfaceNode::Ez21VehicleInterfaceNode(const rclcpp::NodeOptions & options)
: Node("ez21_vehicle_interface", options)
{
  vehicle_id_ = declare_parameter<std::string>("vehicle_id", "default");
  can_interface_ = declare_parameter<std::string>("can_interface", "can1");
  enable_can_io_ = declare_parameter<bool>("enable_can_io", true);
  log_received_messages_ = declare_parameter<bool>("log_received_messages", false);
  control_mode_request_default_success_ =
    declare_parameter<bool>("control_mode_request_default_success", true);
  force_report_autonomous_control_mode_ =
    declare_parameter<bool>("force_report_autonomous_control_mode", false);
  input_qos_depth_ = std::max<int64_t>(1, declare_parameter<int64_t>("input_qos_depth", 1));
  command_period_ms_ = std::max<int64_t>(5, declare_parameter<int64_t>("command_period_ms", 20));
  can_receive_timeout_ms_ =
    std::max<int64_t>(1, declare_parameter<int64_t>("can_receive_timeout_ms", 20));
  wheel_radius_m_ = declare_parameter<double>("wheel_radius_m", 0.2545);
  wheel_base_m_ = declare_parameter<double>("wheel_base_m", 1.1399);
  velocity_zero_threshold_mps_ =
    declare_parameter<double>("velocity_zero_threshold_mps", 0.1);
  steering_center_raw_ = declare_parameter<double>("steering_center_raw", 15750.0);
  steering_counts_per_radian_ =
    declare_parameter<double>("steering_counts_per_radian", 22500.0);
  max_steer_angle_rad_ = declare_parameter<double>("max_steer_angle_rad", kDefaultMaxSteerAngleRad);
  steering_min_speed_deg_per_s_ =
    declare_parameter<double>("steering_min_speed_deg_per_s", 100.0);
  steering_max_speed_deg_per_s_ =
    declare_parameter<double>("steering_max_speed_deg_per_s", 540.0);
  fallback_accel_limit_mps2_ =
    declare_parameter<double>("fallback_accel_limit_mps2", 2.0);
  fallback_brake_limit_mps2_ =
    declare_parameter<double>("fallback_brake_limit_mps2", 3.0);
  overspeed_brake_enabled_ = declare_parameter<bool>("overspeed_brake_enabled", true);
  overspeed_brake_deadband_mps_ =
    declare_parameter<double>("overspeed_brake_deadband_mps", 0.1);
  overspeed_brake_ratio_denominator_min_mps_ =
    declare_parameter<double>("overspeed_brake_ratio_denominator_min_mps", 0.5);
  overspeed_brake_ratio_points_ = declare_parameter<std::vector<double>>(
    "overspeed_brake_ratio_points", std::vector<double>{0.0, 0.05, 0.10, 0.20, 0.35});
  overspeed_brake_cmd_points_ = declare_parameter<std::vector<double>>(
    "overspeed_brake_cmd_points", std::vector<double>{0.0, 0.0, 0.10, 0.35, 0.70});
  drive_current_limit_a_ = declare_parameter<double>("drive_current_limit_a", 80.0);
  drive_max_rpm_ = declare_parameter<double>("drive_max_rpm", 640.0);
  throttle_ad_max_raw_ = declare_parameter<double>("throttle_ad_max_raw", 4095.0);

  if (overspeed_brake_deadband_mps_ < 0.0) {
    throw std::invalid_argument("overspeed_brake_deadband_mps must be non-negative");
  }
  if (overspeed_brake_ratio_denominator_min_mps_ < 0.0) {
    throw std::invalid_argument(
            "overspeed_brake_ratio_denominator_min_mps must be non-negative");
  }
  validate_lookup_table(
    overspeed_brake_ratio_points_, overspeed_brake_cmd_points_, "overspeed_brake_ratio_points",
    "overspeed_brake_cmd_points");

  last_sent_steering_raw_ = static_cast<uint16_t>(std::lround(steering_center_raw_));
  steering_feedback_.raw_angle = last_sent_steering_raw_;

  auto qos = rclcpp::QoS(rclcpp::KeepLast(static_cast<std::size_t>(input_qos_depth_)));

  using std::placeholders::_1;
  using std::placeholders::_2;

  sub_control_cmd_ = create_subscription<Control>(
    "input/control_cmd", qos, std::bind(&Ez21VehicleInterfaceNode::on_control_cmd, this, _1));

  sub_gear_cmd_ = create_subscription<GearCommand>(
    "input/gear_cmd", qos, std::bind(&Ez21VehicleInterfaceNode::on_gear_cmd, this, _1));
  sub_turn_indicators_cmd_ = create_subscription<TurnIndicatorsCommand>(
    "input/turn_indicators_cmd", qos,
    std::bind(&Ez21VehicleInterfaceNode::on_turn_indicators_cmd, this, _1));
  sub_hazard_lights_cmd_ = create_subscription<HazardLightsCommand>(
    "input/hazard_lights_cmd", qos,
    std::bind(&Ez21VehicleInterfaceNode::on_hazard_lights_cmd, this, _1));
  srv_control_mode_request_ = create_service<ControlModeCommand>(
    "input/control_mode_request",
    std::bind(&Ez21VehicleInterfaceNode::on_control_mode_request, this, _1, _2));

  pub_velocity_status_ = create_publisher<VelocityReport>("output/velocity_status", 1);
  pub_steering_status_ = create_publisher<SteeringReport>("output/steering_status", 1);
  pub_gear_status_ = create_publisher<GearReport>("output/gear_status", 1);
  pub_control_mode_ = create_publisher<ControlModeReport>("output/control_mode", 1);

  if (enable_can_io_) {
    start_can_interface();
  } else {
    RCLCPP_WARN(get_logger(), "CAN IO is disabled by parameter; vehicle interface will run dry.");
  }

  timer_ = create_wall_timer(
    std::chrono::milliseconds(command_period_ms_),
    std::bind(&Ez21VehicleInterfaceNode::on_timer, this));

  RCLCPP_INFO(
    get_logger(),
    "ez21_vehicle_interface started for vehicle_id='%s' on %s "
    "(force_report_autonomous_control_mode=%s)",
    vehicle_id_.c_str(), can_interface_.c_str(),
    force_report_autonomous_control_mode_ ? "true" : "false");
}

Ez21VehicleInterfaceNode::~Ez21VehicleInterfaceNode()
{
  stop_can_interface();
}

void Ez21VehicleInterfaceNode::start_can_interface()
{
  try {
    can_sender_ = std::make_unique<drivers::socketcan::SocketCanSender>(
      can_interface_, false, CanId{});
    can_receiver_ = std::make_unique<drivers::socketcan::SocketCanReceiver>(can_interface_, false);
    receiver_running_.store(true);
    receiver_thread_ = std::thread(&Ez21VehicleInterfaceNode::receive_loop, this);
  } catch (const std::exception & ex) {
    can_sender_.reset();
    can_receiver_.reset();
    receiver_running_.store(false);
    RCLCPP_ERROR(
      get_logger(), "Failed to open CAN interface '%s': %s", can_interface_.c_str(), ex.what());
  }
}

void Ez21VehicleInterfaceNode::stop_can_interface()
{
  receiver_running_.store(false);
  if (receiver_thread_.joinable()) {
    receiver_thread_.join();
  }
  can_receiver_.reset();
  can_sender_.reset();
}

void Ez21VehicleInterfaceNode::receive_loop()
{
  while (receiver_running_.load()) {
    if (!can_receiver_) {
      return;
    }

    std::array<uint8_t, kCanDataLength> data{};
    try {
      const auto can_id = can_receiver_->receive(
        data.data(), std::chrono::milliseconds(can_receive_timeout_ms_));
      if (can_id.frame_type() != FrameType::DATA) {
        continue;
      }
      process_can_frame(can_id.identifier(), can_id.is_extended(), data);
    } catch (const SocketCanTimeout &) {
      continue;
    } catch (const std::exception & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "CAN receive error on %s: %s", can_interface_.c_str(),
        ex.what());
    }
  }
}

void Ez21VehicleInterfaceNode::process_can_frame(
  const uint32_t can_id, const bool is_extended, const std::array<uint8_t, 8> & data)
{
  std::lock_guard<std::mutex> lock(data_mutex_);

  if (!is_extended && can_id == kVcuFeedbackId) {
    update_vcu_autonomous_feedback(data);
    return;
  }
  if (is_extended && can_id == kSteeringFeedbackDirectId) {
    update_steering_feedback_direct(data);
    return;
  }
  if (!is_extended && can_id == kSteeringFeedbackRemoteId) {
    update_steering_feedback_remote(data);
    return;
  }
  if (is_extended && can_id == kSteeringFaultFeedbackId) {
    update_steering_fault_state(data);
    return;
  }

  const auto update_drive = [&](const std::array<uint32_t, 4> & ids) {
      for (std::size_t index = 0; index < ids.size(); ++index) {
        if (can_id == ids.at(index)) {
          update_drive_feedback(index, data);
          return true;
        }
      }
      return false;
    };

  if (is_extended && update_drive(kDriveFeedbackDirectIds)) {
    return;
  }
  if (!is_extended) {
    (void)update_drive(kDriveFeedbackRemoteIds);
  }
}

void Ez21VehicleInterfaceNode::update_drive_feedback(
  const std::size_t index, const std::array<uint8_t, 8> & data)
{
  motor_feedbacks_.at(index).valid = true;
  motor_feedbacks_.at(index).wheel_rpm = decode_drive_feedback_rpm(data.at(2));
}

void Ez21VehicleInterfaceNode::update_vcu_autonomous_feedback(const std::array<uint8_t, 8> & data)
{
  vcu_autonomous_feedback_.valid = true;
  vcu_autonomous_feedback_.autonomous_enabled = (data.at(0) & 0x01U) != 0U;
}

void Ez21VehicleInterfaceNode::update_steering_feedback_direct(const std::array<uint8_t, 8> & data)
{
  steering_feedback_.valid = true;
  steering_feedback_.raw_angle = read_le_u16(data, 0U);
  steering_feedback_.steering_tire_angle_rad = vehicle_to_autoware_steering_angle_rad(
    raw_to_steering_angle_rad(
      steering_feedback_.raw_angle, steering_center_raw_, steering_counts_per_radian_));
  steering_feedback_.mode_raw = static_cast<uint8_t>(data.at(6) & 0x0FU);
}

void Ez21VehicleInterfaceNode::update_steering_feedback_remote(const std::array<uint8_t, 8> & data)
{
  steering_feedback_.valid = true;
  steering_feedback_.raw_angle = read_le_u16(data, 0U);
  steering_feedback_.steering_tire_angle_rad = vehicle_to_autoware_steering_angle_rad(
    raw_to_steering_angle_rad(
      steering_feedback_.raw_angle, steering_center_raw_, steering_counts_per_radian_));
  steering_feedback_.mode_raw = static_cast<uint8_t>(data.at(2) & 0x0FU);
  steering_feedback_.fault = steering_feedback_.fault || (data.at(3) != 0U);
}

void Ez21VehicleInterfaceNode::update_steering_fault_state(const std::array<uint8_t, 8> & data)
{
  steering_feedback_.fault = any_nonzero(data);
}

void Ez21VehicleInterfaceNode::on_control_cmd(const Control::ConstSharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_control_cmd_ = *msg;
  }

  if (log_received_messages_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Received control_cmd: velocity=%.3f acceleration=%.3f steering=%.3f",
      msg->longitudinal.velocity, msg->longitudinal.acceleration,
      msg->lateral.steering_tire_angle);
  }

  handle_control_cmd(*msg);
}

void Ez21VehicleInterfaceNode::on_gear_cmd(const GearCommand::ConstSharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_gear_cmd_ = *msg;
  }

  if (log_received_messages_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000, "Received gear_cmd: command=%u",
      static_cast<unsigned int>(msg->command));
  }

  handle_gear_cmd(*msg);
}

void Ez21VehicleInterfaceNode::on_turn_indicators_cmd(
  const TurnIndicatorsCommand::ConstSharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_turn_indicators_cmd_ = *msg;
  }

  if (log_received_messages_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000, "Received turn_indicators_cmd: command=%u",
      static_cast<unsigned int>(msg->command));
  }

  handle_turn_indicators_cmd(*msg);
}

void Ez21VehicleInterfaceNode::on_hazard_lights_cmd(const HazardLightsCommand::ConstSharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_hazard_lights_cmd_ = *msg;
  }

  if (log_received_messages_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000, "Received hazard_lights_cmd: command=%u",
      static_cast<unsigned int>(msg->command));
  }

  handle_hazard_lights_cmd(*msg);
}

void Ez21VehicleInterfaceNode::on_control_mode_request(
  const ControlModeCommand::Request::ConstSharedPtr request,
  const ControlModeCommand::Response::SharedPtr response)
{
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_control_mode_request_ = *request;
  }
  response->success = control_mode_request_default_success_;

  if (log_received_messages_) {
    RCLCPP_INFO(
      get_logger(), "Received control_mode_request: mode=%u",
      static_cast<unsigned int>(request->mode));
  }

  handle_control_mode_request(*request, *response);
}

void Ez21VehicleInterfaceNode::on_timer()
{
  send_command_frames();
  publish_status_messages();
}

void Ez21VehicleInterfaceNode::handle_control_cmd(const Control & /*msg*/)
{
  // The latest control command is converted into drive and steering CAN frames in on_timer().
}

void Ez21VehicleInterfaceNode::handle_gear_cmd(const GearCommand & /*msg*/)
{
  // The latest gear command is applied on the next periodic command frame.
}

void Ez21VehicleInterfaceNode::handle_turn_indicators_cmd(
  const TurnIndicatorsCommand & /*msg*/)
{
  RCLCPP_WARN_THROTTLE(
    get_logger(), *get_clock(), 5000,
    "Turn indicator CAN mapping is not defined in 201整车通讯协议.xlsx; command is ignored.");
}

void Ez21VehicleInterfaceNode::handle_hazard_lights_cmd(
  const HazardLightsCommand & /*msg*/)
{
  RCLCPP_WARN_THROTTLE(
    get_logger(), *get_clock(), 5000,
    "Hazard light CAN mapping is not defined in 201整车通讯协议.xlsx; command is ignored.");
}

void Ez21VehicleInterfaceNode::handle_control_mode_request(
  const ControlModeCommand::Request & request, ControlModeCommand::Response & response)
{
  const bool supported =
    request.mode == ControlModeCommand::Request::AUTONOMOUS ||
    request.mode == ControlModeCommand::Request::AUTONOMOUS_STEER_ONLY ||
    request.mode == ControlModeCommand::Request::AUTONOMOUS_VELOCITY_ONLY ||
    request.mode == ControlModeCommand::Request::MANUAL;

  response.success = response.success && supported;
  if (!supported) {
    RCLCPP_WARN(
      get_logger(), "Unsupported control mode request received: %u",
      static_cast<unsigned int>(request.mode));
    return;
  }

  std::lock_guard<std::mutex> lock(data_mutex_);
  requested_control_mode_ = request.mode;
}

double Ez21VehicleInterfaceNode::resolve_target_velocity_mps() const
{
  if (latest_control_cmd_) {
    return std::abs(latest_control_cmd_->longitudinal.velocity);
  }
  return 0.0;
}

double Ez21VehicleInterfaceNode::resolve_accel_cmd() const
{
  if (!latest_control_cmd_ || fallback_accel_limit_mps2_ <= std::numeric_limits<double>::epsilon()) {
    return 0.0;
  }

  return std::clamp(
    latest_control_cmd_->longitudinal.acceleration / fallback_accel_limit_mps2_, 0.0, 1.0);
}

double Ez21VehicleInterfaceNode::resolve_brake_cmd() const
{
  if (!latest_control_cmd_ || fallback_brake_limit_mps2_ <= std::numeric_limits<double>::epsilon()) {
    return 0.0;
  }

  return std::clamp(
    -latest_control_cmd_->longitudinal.acceleration / fallback_brake_limit_mps2_, 0.0, 1.0);
}

double Ez21VehicleInterfaceNode::resolve_overspeed_brake_cmd() const
{
  if (!overspeed_brake_enabled_ || !latest_control_cmd_) {
    return 0.0;
  }

  const double reference_velocity_mps = std::abs(latest_control_cmd_->longitudinal.velocity);
  const double actual_velocity_mps = std::abs(average_wheel_speed_mps());
  const double raw_overspeed_mps = actual_velocity_mps - reference_velocity_mps;
  if (raw_overspeed_mps <= overspeed_brake_deadband_mps_) {
    return 0.0;
  }

  const double ratio_denominator = std::max(
    reference_velocity_mps, overspeed_brake_ratio_denominator_min_mps_);
  if (ratio_denominator <= std::numeric_limits<double>::epsilon()) {
    return overspeed_brake_cmd_points_.empty() ? 0.0 : overspeed_brake_cmd_points_.back();
  }

  const double overspeed_ratio =
    (raw_overspeed_mps - overspeed_brake_deadband_mps_) / ratio_denominator;
  return std::clamp(
    interpolate_lookup_table(
      overspeed_brake_ratio_points_, overspeed_brake_cmd_points_, overspeed_ratio),
    0.0, 1.0);
}

double Ez21VehicleInterfaceNode::resolve_commanded_steering_tire_angle_rad() const
{
  if (latest_control_cmd_) {
    return latest_control_cmd_->lateral.steering_tire_angle;
  }
  return 0.0;
}

double Ez21VehicleInterfaceNode::resolve_steering_tire_angle_rad() const
{
  if (steering_feedback_.valid) {
    return steering_feedback_.steering_tire_angle_rad;
  }
  if (latest_control_cmd_) {
    return latest_control_cmd_->lateral.steering_tire_angle;
  }
  return 0.0;
}

double Ez21VehicleInterfaceNode::average_wheel_speed_mps() const
{
  double sum_mps = 0.0;
  std::size_t valid_count = 0U;
  for (const auto & feedback : motor_feedbacks_) {
    if (!feedback.valid) {
      continue;
    }
    sum_mps += rpm_to_mps(feedback.wheel_rpm, wheel_radius_m_);
    ++valid_count;
  }
  return valid_count > 0U ? (sum_mps / static_cast<double>(valid_count)) : 0.0;
}

uint8_t Ez21VehicleInterfaceNode::resolve_reported_gear(const double longitudinal_velocity_mps) const
{
  if (std::abs(longitudinal_velocity_mps) > velocity_zero_threshold_mps_) {
    return longitudinal_velocity_mps >= 0.0 ? GearReport::DRIVE : GearReport::REVERSE;
  }

  if (latest_gear_cmd_) {
    return gear_command_to_report(latest_gear_cmd_->command);
  }

  return GearReport::NONE;
}

uint8_t Ez21VehicleInterfaceNode::resolve_requested_control_mode() const
{
  return requested_control_mode_;
}

uint8_t Ez21VehicleInterfaceNode::resolve_reported_control_mode() const
{
  if (force_report_autonomous_control_mode_) {
    return ControlModeReport::AUTONOMOUS;
  }
  if (steering_feedback_.fault) {
    return ControlModeReport::NOT_READY;
  }
  if (vcu_autonomous_feedback_.valid) {
    return vcu_autonomous_feedback_.autonomous_enabled ? ControlModeReport::AUTONOMOUS :
                                                         ControlModeReport::MANUAL;
  }
  if (steering_feedback_.valid) {
    return steering_raw_mode_to_report(steering_feedback_.mode_raw, steering_feedback_.fault);
  }
  return requested_mode_to_report(requested_control_mode_);
}

std::array<uint8_t, 8> Ez21VehicleInterfaceNode::build_vcu_command_frame()
{
  std::array<uint8_t, 8> data{};

  const bool has_control_cmd = latest_control_cmd_.has_value();
  const double signed_target_velocity_mps =
    has_control_cmd ? static_cast<double>(latest_control_cmd_->longitudinal.velocity) : 0.0;
  const double target_velocity_mps = std::abs(signed_target_velocity_mps);
  const double overspeed_brake_cmd = has_control_cmd ? resolve_overspeed_brake_cmd() : 0.0;
  const double brake_cmd =
    has_control_cmd ? std::max(resolve_brake_cmd(), overspeed_brake_cmd) : 0.0;
  const double throttle_percent =
    has_control_cmd && brake_cmd <= 1e-3 ? (10.0 * (target_velocity_mps + 0.17)) : 0.0;
  const double autoware_steering_tire_angle_rad =
    has_control_cmd ? resolve_commanded_steering_tire_angle_rad() : 0.0;
  const double vehicle_steering_tire_angle_rad =
    autoware_to_vehicle_steering_angle_rad(autoware_steering_tire_angle_rad);
  const double steering_ratio =
    max_steer_angle_rad_ > std::numeric_limits<double>::epsilon()
      ? std::clamp(vehicle_steering_tire_angle_rad / max_steer_angle_rad_, -1.0, 1.0)
      : 0.0;
  uint8_t gear_report = GearReport::NONE;

  if (latest_gear_cmd_) {
    gear_report = gear_command_to_report(latest_gear_cmd_->command);
  } else if (signed_target_velocity_mps > velocity_zero_threshold_mps_) {
    gear_report = GearReport::DRIVE;
  } else if (signed_target_velocity_mps < -velocity_zero_threshold_mps_) {
    gear_report = GearReport::REVERSE;
  }

  if (std::abs(vehicle_steering_tire_angle_rad) <= 1e-3) {
    data.at(0) |= 0x08U;
  }
  if (is_drive_gear_report(gear_report)) {
    data.at(0) |= 0x04U;
  }
  if (is_reverse_gear_report(gear_report)) {
    data.at(0) |= 0x02U;
  }
  if (gear_report == GearReport::PARK) {
    data.at(1) |= 0x02U;
  }

  data.at(2) = clamp_ratio_to_percent(throttle_percent, 100.0);
  data.at(3) = clamp_ratio_to_percent(brake_cmd, 1.0);
  data.at(4) = steering_ratio < 0.0 ? clamp_ratio_to_percent(-steering_ratio, 1.0) : 0U;
  data.at(5) = steering_ratio > 0.0 ? clamp_ratio_to_percent(steering_ratio, 1.0) : 0U;
  data.at(6) = 50U;
  data.at(7) = command_heartbeat_++;

  return data;
}

std::array<uint8_t, 8> Ez21VehicleInterfaceNode::build_steering_command_frame(
  const double steering_tire_angle_rad, const uint8_t mode_request)
{
  std::array<uint8_t, 8> data{};
  const uint16_t raw_steering = steering_angle_rad_to_raw(
    autoware_to_vehicle_steering_angle_rad(steering_tire_angle_rad), steering_center_raw_,
    steering_counts_per_radian_, max_steer_angle_rad_);
  const double command_period_s = static_cast<double>(command_period_ms_) / 1000.0;
  const double delta_angle_rad =
    std::abs(static_cast<double>(raw_steering) - static_cast<double>(last_sent_steering_raw_)) /
    std::max(steering_counts_per_radian_, std::numeric_limits<double>::epsilon());
  const double speed_deg_per_s =
    (delta_angle_rad / std::max(command_period_s, 1e-3)) * (180.0 / kPi);

  write_le_u16(data, 0U, raw_steering);
  data.at(2) = is_autonomous_request(mode_request) ? kSteeringAutonomousMode : kSteeringEpsMode;
  data.at(3) = 0x00U;
  data.at(4) = steering_speed_deg_per_s_to_raw(
    speed_deg_per_s, steering_min_speed_deg_per_s_, steering_max_speed_deg_per_s_);
  data.at(5) = 0x00U;
  data.at(6) = 0x00U;
  data.at(7) = 0x00U;

  last_sent_steering_raw_ = raw_steering;
  return data;
}

std::array<uint8_t, 8> Ez21VehicleInterfaceNode::build_drive_command_frame(
  const double target_velocity_mps, const double accel_cmd, const double brake_cmd,
  const uint8_t gear_command, const uint8_t mode_request) const
{
  std::array<uint8_t, 8> data{};
  const uint8_t gear_report = gear_command_to_report(gear_command);
  const bool drive_enabled =
    is_autonomous_request(mode_request) && is_motion_enabled_gear(gear_report);
  const double speed_limit_rpm = drive_enabled && brake_cmd <= 1e-3
                                   ? std::clamp(
                                       mps_to_rpm(target_velocity_mps, wheel_radius_m_), 0.0,
                                       drive_max_rpm_)
                                   : 0.0;
  const double throttle_raw = drive_enabled && brake_cmd <= 1e-3
                                ? std::clamp(accel_cmd * throttle_ad_max_raw_, 0.0, throttle_ad_max_raw_)
                                : 0.0;

  data.at(0) = 0x10U;
  data.at(1) = drive_enabled ? kDriveEnableValue : 0x00U;
  data.at(2) = static_cast<uint8_t>(
    std::lround(std::clamp(drive_current_limit_a_, 0.0, 255.0)));
  write_be_u16(data, 3U, static_cast<uint16_t>(std::lround(speed_limit_rpm)));
  write_be_u16(data, 5U, static_cast<uint16_t>(std::lround(throttle_raw)));
  data.at(7) = is_reverse_gear_report(gear_report) ? 0x01U : 0x00U;

  return data;
}

void Ez21VehicleInterfaceNode::send_vcu_command_frame()
{
  if (!can_sender_) {
    return;
  }

  const auto vcu_data = build_vcu_command_frame();
  const CanId can_id(kVcuCommandId, 0U, FrameType::DATA, StandardFrame);
  can_sender_->send(vcu_data.data(), vcu_data.size(), can_id, std::chrono::milliseconds(5));
}

void Ez21VehicleInterfaceNode::send_steering_command_frame()
{
  if (!can_sender_) {
    return;
  }

  const auto steering_data = build_steering_command_frame(
    resolve_steering_tire_angle_rad(), resolve_requested_control_mode());
  const CanId can_id(kSteeringCommandId, 0U, FrameType::DATA, ExtendedFrame);

  can_sender_->send(
    steering_data.data(), steering_data.size(), can_id, std::chrono::milliseconds(5));
}

void Ez21VehicleInterfaceNode::send_drive_command_frames()
{
  if (!can_sender_) {
    return;
  }

  const uint8_t gear_command = latest_gear_cmd_ ? latest_gear_cmd_->command : GearCommand::NONE;
  const auto drive_data = build_drive_command_frame(
    resolve_target_velocity_mps(), resolve_accel_cmd(), resolve_brake_cmd(), gear_command,
    resolve_requested_control_mode());

  for (const auto command_id : kDriveCommandIds) {
    const CanId can_id(command_id, 0U, FrameType::DATA, ExtendedFrame);
    can_sender_->send(
      drive_data.data(), drive_data.size(), can_id, std::chrono::milliseconds(5));
  }
}

void Ez21VehicleInterfaceNode::send_command_frames()
{
  if (!enable_can_io_ || !can_sender_) {
    return;
  }

  try {
    std::lock_guard<std::mutex> lock(data_mutex_);
    send_vcu_command_frame();
  } catch (const std::exception & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "CAN send error on %s: %s", can_interface_.c_str(),
      ex.what());
  }
}

void Ez21VehicleInterfaceNode::publish_status_messages()
{
  VelocityReport velocity_msg;
  SteeringReport steering_msg;
  GearReport gear_msg;
  ControlModeReport control_mode_msg;

  const auto stamp = get_clock()->now();

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    const double longitudinal_velocity_mps = average_wheel_speed_mps();
    const double steering_tire_angle_rad = resolve_steering_tire_angle_rad();

    velocity_msg.header.stamp = stamp;
    velocity_msg.header.frame_id = "base_link";
    velocity_msg.longitudinal_velocity = static_cast<float>(longitudinal_velocity_mps);
    velocity_msg.lateral_velocity = 0.0F;
    velocity_msg.heading_rate = static_cast<float>(
      wheel_base_m_ > std::numeric_limits<double>::epsilon()
        ? longitudinal_velocity_mps * std::tan(steering_tire_angle_rad) / wheel_base_m_
        : 0.0);

    steering_msg.stamp = stamp;
    steering_msg.steering_tire_angle = static_cast<float>(steering_tire_angle_rad);

    gear_msg.stamp = stamp;
    gear_msg.report = resolve_reported_gear(longitudinal_velocity_mps);

    control_mode_msg.stamp = stamp;
    control_mode_msg.mode = resolve_reported_control_mode();
  }

  pub_velocity_status_->publish(velocity_msg);
  pub_steering_status_->publish(steering_msg);
  pub_gear_status_->publish(gear_msg);
  pub_control_mode_->publish(control_mode_msg);
}

}  // namespace ez21_vehicle_launch
