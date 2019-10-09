#include "ros_spatial_utils/map.h"

#include <gtest/gtest.h>

using namespace ros_spatial_utils;

static nav_msgs::OccupancyGrid makeTestMap()
{
  nav_msgs::OccupancyGrid map;
  map.info.resolution = 0.5;
  map.info.width = 5;
  map.info.height = 5;
  map.data = {
    -1, -1, -1, -1, 100, 100, 100, 100, 100, 100, 100, 0, 0, 0, 0, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  };
  return map;
}

TEST(TestMap, computeStateMap)
{
  const nav_msgs::OccupancyGrid map = makeTestMap();
  MapStateContainer map_state;

  MapStateContainer expected_state(5, 5);
  expected_state << MapState::Unknown, MapState::Unknown, MapState::Unknown, MapState::Unknown, MapState::Occupied,
      MapState::Occupied, MapState::Occupied, MapState::Occupied, MapState::Occupied, MapState::Occupied,
      MapState::Occupied, MapState::Free, MapState::Free, MapState::Free, MapState::Free, MapState::Occupied,
      MapState::Free, MapState::Free, MapState::Free, MapState::Free, MapState::Free, MapState::Free, MapState::Free,
      MapState::Free, MapState::Free;

  computeStateMap(map, &map_state);
  for (Eigen::Index y = 0; y < expected_state.cols(); y++)
  {
    for (Eigen::Index x = 0; x < expected_state.rows(); x++)
    {
      EXPECT_EQ(expected_state(y, x), map_state(y, x)) << "x=" << x << " y=" << y;
    }
  }
}
