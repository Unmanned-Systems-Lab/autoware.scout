#include "scout_ht_planning/velocity_module.hpp"

#include <autoware/motion_utils/marker/marker_helper.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <algorithm>
#include <cmath>

namespace scout_ht_planning
{
namespace mu = autoware::motion_utils;
using Result = autoware::motion_velocity_planner::VelocityPlanningResult;
void VelocityModule::init(rclcpp::Node & node, const std::string & name)
{
  node_ = &node;
  name_ = name;
  logger_ = node.get_logger();
  guard_ = std::make_unique<CostmapGuard>(node);
  status_ = node.create_publisher<std_msgs::msg::String>("~/scout_ht/status", 1);
  virtual_wall_publisher_ =
    node.create_publisher<visualization_msgs::msg::MarkerArray>("~/scout_ht/virtual_wall", 1);
}
Result VelocityModule::plan(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & raw,
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> &,
  std::shared_ptr<const autoware::motion_velocity_planner::PlannerData> data)
{
  Result result;
  if (raw.empty()) return result;
  geometry_msgs::msg::Point start = raw.front().pose.position;
  Collision collision;
  double stop_distance = 0;
  double response_time = guard_->latency;
  try {
    if (!data || raw.size() < 2) throw std::invalid_argument("INVALID_TRAJECTORY");
    const auto & odom = data->current_odometry;
    const double age = node_->now().seconds() - rclcpp::Time(odom.header.stamp).seconds();
    if (
      odom.header.frame_id != "map" || odom.child_frame_id != "base_link" || age < -0.05 ||
      age > 0.5 || rclcpp::Time(odom.header.stamp).nanoseconds() <= 0 ||
      !std::isfinite(odom.twist.twist.linear.x))
      throw std::invalid_argument("INVALID_ODOMETRY");
    const auto ego = planar_pose(odom.pose.pose);
    start = mu::calcLongitudinalOffsetPoint(raw, odom.pose.pose.position, 0.0).value_or(start);
    const auto snapshot = guard_->snapshot("map");
    if (!snapshot.grid) throw std::invalid_argument(snapshot.reason);
    response_time +=
      std::max(0.0, node_->now().seconds() - rclcpp::Time(snapshot.grid->header.stamp).seconds());
    if (!data->is_driving_forward) throw std::invalid_argument("REVERSE_UNSUPPORTED");
    const auto segment = data->find_segment_index(raw, odom.pose.pose);
    std::vector<Pose2> poses{ego};
    for (size_t i = segment + 1; i < raw.size(); ++i) poses.push_back(planar_pose(raw[i].pose));
    const auto & info = data->vehicle_info_;
    const Footprint footprint{
      info.max_longitudinal_offset_m, info.min_longitudinal_offset_m, info.max_lateral_offset_m,
      info.min_lateral_offset_m};
    collision = check_path(*snapshot.grid, poses, footprint);
    if (!guard_->current(snapshot)) throw std::invalid_argument("MAP_EXPIRED_DURING_CHECK");
    if (collision.blocked) {
      // The actual-pose join can be longer than its projection onto the path.
      // Never use that extra lateral distance as extra braking clearance.
      const double join = std::hypot(poses[1].x - ego.x, poses[1].y - ego.y);
      const double projected = std::max(0.0, mu::calcSignedArcLength(raw, start, segment + 1));
      stop_distance =
        std::max(0.0, collision.distance - std::max(0.0, join - projected) - guard_->stop_margin);
    }
  } catch (const std::exception & error) {
    collision = {true, 0, error.what()};
  }
  std_msgs::msg::String status;
  status.data = collision.reason + " stop_distance=" + std::to_string(stop_distance);
  status_->publish(status);
  auto walls = mu::createDeletedStopVirtualWallMarker(node_->now(), 0);
  if (collision.blocked) {
    const auto stop = mu::calcLongitudinalOffsetPoint(raw, start, stop_distance).value_or(start);
    result.stop_points.push_back(stop);
    for (double d = 0; d < stop_distance; d += 0.2) {
      const double end = std::min(stop_distance, d + 0.2);
      const auto a = mu::calcLongitudinalOffsetPoint(raw, start, d);
      const auto b = mu::calcLongitudinalOffsetPoint(raw, start, end);
      if (a && b)
        result.slowdown_intervals.emplace_back(
          *a, *b, safe_speed(stop_distance - end, guard_->deceleration, response_time));
    }
    auto pose = raw.front().pose;
    pose.position = stop;
    walls =
      mu::createStopVirtualWallMarker(pose, "scout_ht: " + collision.reason, node_->now(), 0, 0.0);
  }
  virtual_wall_publisher_->publish(walls);
  return result;
}
}  // namespace scout_ht_planning
PLUGINLIB_EXPORT_CLASS(
  scout_ht_planning::VelocityModule, autoware::motion_velocity_planner::PluginModuleInterface)
