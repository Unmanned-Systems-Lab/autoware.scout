#ifndef EZ21_VEHICLE_LAUNCH__EZ21_VEHICLE_INTERFACE_NODE_HPP_
#define EZ21_VEHICLE_LAUNCH__EZ21_VEHICLE_INTERFACE_NODE_HPP_

#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <autoware_vehicle_msgs/srv/control_mode_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ros2_socketcan/socket_can_receiver.hpp>
#include <ros2_socketcan/socket_can_sender.hpp>

#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace ez21_vehicle_launch
{

class Ez21VehicleInterfaceNode : public rclcpp::Node
{
public:
  explicit Ez21VehicleInterfaceNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~Ez21VehicleInterfaceNode() override;

private:
  using Control = autoware_control_msgs::msg::Control;
  using ControlModeCommand = autoware_vehicle_msgs::srv::ControlModeCommand;
  using ControlModeReport = autoware_vehicle_msgs::msg::ControlModeReport;
  using GearCommand = autoware_vehicle_msgs::msg::GearCommand;
  using GearReport = autoware_vehicle_msgs::msg::GearReport;
  using HazardLightsCommand = autoware_vehicle_msgs::msg::HazardLightsCommand;
  using SteeringReport = autoware_vehicle_msgs::msg::SteeringReport;
  using TurnIndicatorsCommand = autoware_vehicle_msgs::msg::TurnIndicatorsCommand;
  using VelocityReport = autoware_vehicle_msgs::msg::VelocityReport;

  struct MotorFeedback
  {
    bool valid{false};
    double wheel_rpm{0.0};
  };

  struct SteeringFeedback
  {
    bool valid{false};
    uint16_t raw_angle{0U};
    double steering_tire_angle_rad{0.0};
    uint8_t mode_raw{0U};
    bool fault{false};
  };

  struct VcuAutonomousFeedback
  {
    bool valid{false};
    bool autonomous_enabled{false};
  };

  void on_control_cmd(const Control::ConstSharedPtr msg);
  void on_gear_cmd(const GearCommand::ConstSharedPtr msg);
  void on_turn_indicators_cmd(const TurnIndicatorsCommand::ConstSharedPtr msg);
  void on_hazard_lights_cmd(const HazardLightsCommand::ConstSharedPtr msg);
  void on_control_mode_request(
    const ControlModeCommand::Request::ConstSharedPtr request,
    const ControlModeCommand::Response::SharedPtr response);
  void on_timer();

  void handle_control_cmd(const Control & msg);
  void handle_gear_cmd(const GearCommand & msg);
  void handle_turn_indicators_cmd(const TurnIndicatorsCommand & msg);
  void handle_hazard_lights_cmd(const HazardLightsCommand & msg);
  void handle_control_mode_request(
    const ControlModeCommand::Request & request,
    ControlModeCommand::Response & response);

  void start_can_interface();
  void stop_can_interface();
  void receive_loop();
  void process_can_frame(uint32_t can_id, bool is_extended, const std::array<uint8_t, 8> & data);
  void publish_status_messages();
  void send_command_frames();
  void send_vcu_command_frame();
  void send_steering_command_frame();
  void send_drive_command_frames();

  std::array<uint8_t, 8> build_vcu_command_frame();
  std::array<uint8_t, 8> build_steering_command_frame(
    double steering_tire_angle_rad, uint8_t mode_request);
  std::array<uint8_t, 8> build_drive_command_frame(
    double target_velocity_mps, double accel_cmd, double brake_cmd, uint8_t gear_command,
    uint8_t mode_request) const;

  double resolve_target_velocity_mps() const;
  double resolve_accel_cmd() const;
  double resolve_brake_cmd() const;
  double resolve_overspeed_brake_cmd() const;
  double resolve_commanded_steering_tire_angle_rad() const;
  double resolve_steering_tire_angle_rad() const;
  double average_wheel_speed_mps() const;
  uint8_t resolve_reported_gear(double longitudinal_velocity_mps) const;
  uint8_t resolve_reported_control_mode() const;
  uint8_t resolve_requested_control_mode() const;
  void update_drive_feedback(std::size_t index, const std::array<uint8_t, 8> & data);
  void update_vcu_autonomous_feedback(const std::array<uint8_t, 8> & data);
  void update_steering_feedback_direct(const std::array<uint8_t, 8> & data);
  void update_steering_feedback_remote(const std::array<uint8_t, 8> & data);
  void update_steering_fault_state(const std::array<uint8_t, 8> & data);

  std::string vehicle_id_;
  std::string can_interface_;
  bool enable_can_io_;
  bool log_received_messages_;
  bool control_mode_request_default_success_;
  bool force_report_autonomous_control_mode_;
  int64_t input_qos_depth_;
  int64_t command_period_ms_;
  int64_t can_receive_timeout_ms_;
  double wheel_radius_m_;
  double wheel_base_m_;
  double velocity_zero_threshold_mps_;
  double steering_center_raw_;
  double steering_counts_per_radian_;
  double max_steer_angle_rad_;
  double steering_min_speed_deg_per_s_;
  double steering_max_speed_deg_per_s_;
  double fallback_accel_limit_mps2_;
  double fallback_brake_limit_mps2_;
  bool overspeed_brake_enabled_;
  double overspeed_brake_deadband_mps_;
  double overspeed_brake_ratio_denominator_min_mps_;
  std::vector<double> overspeed_brake_ratio_points_;
  std::vector<double> overspeed_brake_cmd_points_;
  double drive_current_limit_a_;
  double drive_max_rpm_;
  double throttle_ad_max_raw_;

  mutable std::mutex data_mutex_;
  std::array<MotorFeedback, 4> motor_feedbacks_{};
  SteeringFeedback steering_feedback_{};
  VcuAutonomousFeedback vcu_autonomous_feedback_{};
  uint8_t requested_control_mode_{ControlModeCommand::Request::MANUAL};
  uint8_t command_heartbeat_{0U};
  uint16_t last_sent_steering_raw_{0U};

  rclcpp::Subscription<Control>::SharedPtr sub_control_cmd_;
  rclcpp::Subscription<GearCommand>::SharedPtr sub_gear_cmd_;
  rclcpp::Subscription<TurnIndicatorsCommand>::SharedPtr sub_turn_indicators_cmd_;
  rclcpp::Subscription<HazardLightsCommand>::SharedPtr sub_hazard_lights_cmd_;
  rclcpp::Service<ControlModeCommand>::SharedPtr srv_control_mode_request_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<VelocityReport>::SharedPtr pub_velocity_status_;
  rclcpp::Publisher<SteeringReport>::SharedPtr pub_steering_status_;
  rclcpp::Publisher<GearReport>::SharedPtr pub_gear_status_;
  rclcpp::Publisher<ControlModeReport>::SharedPtr pub_control_mode_;

  std::optional<Control> latest_control_cmd_;
  std::optional<GearCommand> latest_gear_cmd_;
  std::optional<TurnIndicatorsCommand> latest_turn_indicators_cmd_;
  std::optional<HazardLightsCommand> latest_hazard_lights_cmd_;
  std::optional<ControlModeCommand::Request> latest_control_mode_request_;

  std::unique_ptr<drivers::socketcan::SocketCanSender> can_sender_;
  std::unique_ptr<drivers::socketcan::SocketCanReceiver> can_receiver_;
  std::atomic<bool> receiver_running_{false};
  std::thread receiver_thread_;
};

}  // namespace ez21_vehicle_launch

#endif  // EZ21_VEHICLE_LAUNCH__EZ21_VEHICLE_INTERFACE_NODE_HPP_
