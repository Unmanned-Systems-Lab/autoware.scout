#include "scout_ht_planning/costmap.hpp"

#include <boost/geometry.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace scout_ht_planning
{
namespace
{
namespace bg = boost::geometry;
using Point = bg::model::d2::point_xy<double>;
using Polygon = bg::model::polygon<Point>;
using Points = bg::model::multi_point<Point>;
constexpr double pi = 3.14159265358979323846;
bool finite(const Pose2 & p)
{
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.yaw) && std::abs(p.x) < 1e9 &&
         std::abs(p.y) < 1e9 && std::abs(p.yaw) < 1e6;
}
double angle(double value)
{
  return std::remainder(value, 2 * pi);
}
Pose2 interpolate(const Pose2 & a, const Pose2 & b, double t)
{
  return {a.x + t * (b.x - a.x), a.y + t * (b.y - a.y), a.yaw + t * angle(b.yaw - a.yaw)};
}
Pose2 local_pose(const Pose2 & p, const Pose2 & origin)
{
  const auto c = std::cos(origin.yaw), s = std::sin(origin.yaw);
  const auto x = p.x - origin.x, y = p.y - origin.y;
  return {c * x + s * y, -s * x + c * y, p.yaw - origin.yaw};
}
void add_corners(Points & points, const Pose2 & p, const Footprint & f)
{
  const auto c = std::cos(p.yaw), s = std::sin(p.yaw);
  for (double x : {f.rear, f.front}) {
    for (double y : {f.right, f.left}) {
      points.emplace_back(p.x + c * x - s * y, p.y + s * x + c * y);
    }
  }
}
std::string swept_collision(
  const nav_msgs::msg::OccupancyGrid & grid, const Pose2 & a, const Pose2 & b, const Footprint & f)
{
  Points corners;
  add_corners(corners, a, f);
  add_corners(corners, b, f);
  Polygon hull;
  bg::convex_hull(corners, hull);
  // Every rotating corner stays within this distance of its endpoint chord.
  // Hull + sagitta covers continuous translation/rotation, not just samples.
  const double radius = std::hypot(
    std::max(std::abs(f.front), std::abs(f.rear)), std::max(std::abs(f.left), std::abs(f.right)));
  const double padding = radius * (1 - std::cos(std::abs(angle(b.yaw - a.yaw)) / 2)) + 1e-9;
  bg::model::box<Point> box;
  bg::envelope(hull, box);
  const double min_x = box.min_corner().x() - padding;
  const double min_y = box.min_corner().y() - padding;
  const double max_x = box.max_corner().x() + padding;
  const double max_y = box.max_corner().y() + padding;
  const double res = grid.info.resolution;
  if (min_x < 0 || min_y < 0 || max_x >= grid.info.width * res || max_y >= grid.info.height * res)
    return "OUTSIDE_MAP";
  const auto x0 = static_cast<int>(std::floor(min_x / res));
  const auto y0 = static_cast<int>(std::floor(min_y / res));
  const auto x1 = static_cast<int>(std::floor(max_x / res));
  const auto y1 = static_cast<int>(std::floor(max_y / res));
  for (int y = y0; y <= y1; ++y) {
    for (int x = x0; x <= x1; ++x) {
      // This is a binary fused grid: only an explicit zero is free.
      if (grid.data[static_cast<size_t>(y) * grid.info.width + x] == 0) continue;
      Polygon cell;
      bg::append(cell, Point(x * res, y * res));
      bg::append(cell, Point((x + 1) * res, y * res));
      bg::append(cell, Point((x + 1) * res, (y + 1) * res));
      bg::append(cell, Point(x * res, (y + 1) * res));
      bg::correct(cell);
      if (bg::distance(hull, cell) <= padding) return "BLOCKED_CELL";
    }
  }
  return {};
}
}  // namespace

Pose2 planar_pose(const geometry_msgs::msg::Pose & pose)
{
  const auto & q = pose.orientation;
  const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  if (!std::isfinite(norm) || std::abs(norm - 1) > 1e-3 || !std::isfinite(pose.position.z))
    throw std::invalid_argument("Invalid pose");
  Pose2 p{
    pose.position.x, pose.position.y,
    std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))};
  if (!finite(p)) throw std::invalid_argument("Nonfinite pose");
  return p;
}

bool valid_grid(const nav_msgs::msg::OccupancyGrid & grid)
{
  if (
    grid.header.frame_id.empty() || !std::isfinite(grid.info.resolution) ||
    grid.info.resolution < 0.01 || grid.info.resolution > 2 || grid.info.width == 0 ||
    grid.info.height == 0 || static_cast<uint64_t>(grid.info.width) * grid.info.height > 4000000 ||
    grid.data.size() != static_cast<size_t>(grid.info.width) * grid.info.height)
    return false;
  if (
    std::abs(grid.info.origin.orientation.x) > 1e-3 ||
    std::abs(grid.info.origin.orientation.y) > 1e-3)
    return false;
  try {
    planar_pose(grid.info.origin);
  } catch (const std::exception &) {
    return false;
  }
  return true;
}

Collision check_path(
  const nav_msgs::msg::OccupancyGrid & grid, const std::vector<Pose2> & path,
  const Footprint & footprint, double max_distance)
{
  if (
    !valid_grid(grid) || path.empty() || std::isnan(max_distance) || max_distance < 0 ||
    !std::isfinite(footprint.front) || !std::isfinite(footprint.rear) ||
    !std::isfinite(footprint.left) || !std::isfinite(footprint.right) ||
    footprint.front <= footprint.rear || footprint.left <= footprint.right)
    return {};
  for (const auto & p : path)
    if (!finite(p)) return {};
  const auto origin = planar_pose(grid.info.origin);
  auto previous = local_pose(path.front(), origin);
  auto reason = swept_collision(grid, previous, previous, footprint);
  if (!reason.empty()) return {true, 0, reason};
  double distance = 0;
  for (size_t i = 1; i < path.size() && distance < max_distance; ++i) {
    const auto start = local_pose(path[i - 1], origin);
    const auto end = local_pose(path[i], origin);
    const double length = std::hypot(end.x - start.x, end.y - start.y);
    const double fraction = length > 0 ? std::min(1.0, (max_distance - distance) / length) : 1;
    const auto target = interpolate(start, end, fraction);
    const double required_steps = std::ceil(std::max(
      length * fraction / std::min(0.1, grid.info.resolution * 0.5),
      std::abs(angle(target.yaw - start.yaw)) / (pi / 36)));
    if (!std::isfinite(required_steps) || required_steps > 100000) return {};
    const int steps = std::max(1, static_cast<int>(required_steps));
    for (int k = 1; k <= steps; ++k) {
      const auto next = interpolate(start, target, static_cast<double>(k) / steps);
      reason = swept_collision(grid, previous, next, footprint);
      if (!reason.empty()) return {true, distance, reason};
      distance += std::hypot(next.x - previous.x, next.y - previous.y);
      previous = next;
    }
  }
  return {false, distance, "CLEAR"};
}

double safe_speed(double distance, double deceleration, double latency)
{
  if (
    !std::isfinite(distance) || !std::isfinite(deceleration) || !std::isfinite(latency) ||
    distance <= 0 || deceleration <= 0 || latency < 0)
    return 0;
  return std::sqrt(deceleration * deceleration * latency * latency + 2 * deceleration * distance) -
         deceleration * latency;
}
}  // namespace scout_ht_planning
