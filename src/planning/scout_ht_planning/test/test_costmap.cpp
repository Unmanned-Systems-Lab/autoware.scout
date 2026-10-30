#include "test_grid.hpp"

#include <gtest/gtest.h>

#include <cmath>

using namespace scout_ht_planning;
TEST(Costmap, ClearDoesNotCreateStop)
{
  EXPECT_FALSE(check_path(free_grid(), {{0, 0, 0}, {5, 0, 0}}, test_footprint()).blocked);
}
TEST(Costmap, SparseEndpointsCannotJumpOverBlackCell)
{
  auto grid = free_grid();
  obstacle(grid, 2, 0);
  auto hit = check_path(grid, {{0, 0, 0}, {5, 0, 0}}, test_footprint());
  EXPECT_TRUE(hit.blocked);
  EXPECT_GT(hit.distance, 1.2);
  EXPECT_LT(hit.distance, 1.6);
}
TEST(Costmap, VehicleWidthNotJustCenterline)
{
  auto grid = free_grid();
  obstacle(grid, 2, 0.25);
  EXPECT_TRUE(check_path(grid, {{0, 0, 0}, {5, 0, 0}}, test_footprint()).blocked);
}
TEST(Costmap, BlackOutsideSweptBodyDoesNotStop)
{
  auto grid = free_grid();
  obstacle(grid, 2, 0.8);
  EXPECT_FALSE(check_path(grid, {{0, 0, 0}, {5, 0, 0}}, test_footprint()).blocked);
}
TEST(Costmap, UnknownAndNonbinaryCostsAreNeverWhite)
{
  for (int8_t v : {-1, 1, 50, 100}) {
    auto grid = free_grid();
    obstacle(grid, 2, 0, v);
    EXPECT_TRUE(check_path(grid, {{0, 0, 0}, {5, 0, 0}}, test_footprint()).blocked);
  }
}
TEST(Costmap, MissingCoverageStopsBeforeMapEdge)
{
  auto hit = check_path(free_grid(), {{0, 0, 0}, {12, 0, 0}}, test_footprint());
  EXPECT_TRUE(hit.blocked);
  EXPECT_EQ(hit.reason, "OUTSIDE_MAP");
  EXPECT_LT(hit.distance, 9.6);
}
TEST(Costmap, AlreadyBlockedHasZeroClearance)
{
  auto grid = free_grid();
  obstacle(grid, 0, 0);
  auto hit = check_path(grid, {{0, 0, 0}, {5, 0, 0}}, test_footprint());
  EXPECT_TRUE(hit.blocked);
  EXPECT_EQ(hit.distance, 0);
}
TEST(Costmap, RotationSweepIncludesIntermediateCorners)
{
  auto grid = free_grid();
  obstacle(grid, 0.4, 0.4);
  const Footprint body{0.6, -0.6, 0.1, -0.1};
  EXPECT_FALSE(check_path(grid, {{0, 0, 0}}, body).blocked);
  EXPECT_FALSE(check_path(grid, {{0, 0, 1.57079632679}}, body).blocked);
  EXPECT_TRUE(check_path(grid, {{0, 0, 0}, {0, 0, 1.57079632679}}, body).blocked);
}
TEST(Costmap, RotatedGridUsesOriginPose)
{
  auto grid = free_grid();
  grid.info.origin.position.x = 5;
  grid.info.origin.orientation.z = std::sin(0.5);
  grid.info.origin.orientation.w = std::cos(0.5);
  auto world = [&](double x, double y) -> Pose2 {
    return {
      5 + std::cos(1.0) * x - std::sin(1.0) * y, -5 + std::sin(1.0) * x + std::cos(1.0) * y, 1.0};
  };
  grid.data[50 * grid.info.width + 70] = 100;
  EXPECT_TRUE(check_path(grid, {world(5, 5), world(10, 5)}, test_footprint()).blocked);
}
TEST(Costmap, OneSolveCanLimitTheExecutablePrefix)
{
  auto grid = free_grid();
  obstacle(grid, 5, 0);
  EXPECT_FALSE(check_path(grid, {{0, 0, 0}, {8, 0, 0}}, test_footprint(), 2).blocked);
  EXPECT_TRUE(check_path(grid, {{0, 0, 0}, {8, 0, 0}}, test_footprint()).blocked);
}
TEST(Costmap, InvalidGeometryAndEmptyPathsAreBlocked)
{
  auto grid = free_grid();
  grid.data.pop_back();
  EXPECT_TRUE(check_path(grid, {{0, 0, 0}}, test_footprint()).blocked);
  EXPECT_TRUE(check_path(free_grid(), {}, test_footprint()).blocked);
  grid = free_grid();
  grid.info.origin.orientation.w = 0;
  EXPECT_FALSE(valid_grid(grid));
}
TEST(Costmap, NonfinitePosesAndInvalidFootprintsAreBlocked)
{
  EXPECT_TRUE(check_path(free_grid(), {{NAN, 0, 0}}, test_footprint()).blocked);
  EXPECT_TRUE(check_path(free_grid(), {{0, 0, 0}}, {0, 0, 0, 0}).blocked);
}
TEST(Costmap, BrakingSpeedIncludesLatencyAndNeverExceedsAvailableDistance)
{
  for (double distance : {0.0, 0.1, 1.0, 5.0}) {
    const double v = safe_speed(distance, .5, .3);
    EXPECT_LE(v * .3 + v * v / (2 * .5), distance + 1e-12);
  }
  EXPECT_EQ(safe_speed(1, 0, .3), 0);
}
