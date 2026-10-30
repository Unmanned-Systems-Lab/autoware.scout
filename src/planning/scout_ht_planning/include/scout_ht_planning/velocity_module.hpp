#ifndef SCOUT_HT_PLANNING__VELOCITY_MODULE_HPP_
#define SCOUT_HT_PLANNING__VELOCITY_MODULE_HPP_
#include "scout_ht_planning/guard.hpp"

#include <autoware/motion_velocity_planner_common/plugin_module_interface.hpp>

#include <std_msgs/msg/string.hpp>

namespace scout_ht_planning
{
class VelocityModule : public autoware::motion_velocity_planner::PluginModuleInterface
{
public:
  void init(rclcpp::Node & node, const std::string & module_name) override;
  autoware::motion_velocity_planner::RequiredSubscriptionInfo getRequiredSubscriptions()
    const override
  {
    return {};
  }
  void update_parameters(const std::vector<rclcpp::Parameter> &) override {}
  std::string get_module_name() const override { return name_; }
  std::string get_short_module_name() const override { return "scout_ht"; }
  autoware::motion_velocity_planner::VelocityPlanningResult plan(
    const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & raw,
    const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & smoothed,
    std::shared_ptr<const autoware::motion_velocity_planner::PlannerData> data) override;

private:
  rclcpp::Node * node_{};
  std::string name_;
  std::unique_ptr<CostmapGuard> guard_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
};
}  // namespace scout_ht_planning
#endif
