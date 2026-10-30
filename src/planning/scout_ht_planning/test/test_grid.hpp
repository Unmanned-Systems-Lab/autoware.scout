#ifndef SCOUT_HT_PLANNING_TEST_GRID_HPP_
#define SCOUT_HT_PLANNING_TEST_GRID_HPP_
#include "scout_ht_planning/costmap.hpp"
inline nav_msgs::msg::OccupancyGrid free_grid()
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = "map";
  grid.info.resolution = 0.1;
  grid.info.width = 150;
  grid.info.height = 100;
  grid.info.origin.position.x = -5;
  grid.info.origin.position.y = -5;
  grid.info.origin.orientation.w = 1;
  grid.data.assign(grid.info.width * grid.info.height, 0);
  return grid;
}
inline void obstacle(nav_msgs::msg::OccupancyGrid & grid, double x, double y, int8_t value = 100)
{
  const int ix = static_cast<int>((x - grid.info.origin.position.x) / grid.info.resolution);
  const int iy = static_cast<int>((y - grid.info.origin.position.y) / grid.info.resolution);
  grid.data.at(iy * grid.info.width + ix) = value;
}
inline scout_ht_planning::Footprint test_footprint()
{
  return {0.5, -0.5, 0.3, -0.3};
}
#endif
