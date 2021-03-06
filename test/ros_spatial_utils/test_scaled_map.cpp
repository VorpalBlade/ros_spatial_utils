#include "ros_spatial_utils/scaled_map.h"

#include <gtest/gtest.h>

using namespace ros_spatial_utils;

template <typename T>
std::ostream& operator<<(std::ostream& os, const Eigen::Matrix<T, 2, 1>& v)
{
  os << "[" << v(0) << ", " << v(1) << "]";
  return os;
}

template <typename T>
std::ostream& operator<<(std::ostream& os, const Eigen::Matrix<T, 3, 1>& v)
{
  os << "[" << v(0) << ", " << v(1) << ", " << v(2) << "]";
  return os;
}

TEST(TestScaledInfiniteMapLogic, transformations)
{
  using Map2d = ScaledInfiniteMapLogic<2>;
  using Map3d = ScaledInfiniteMapLogic<3>;
  Map2d map_2d;
  Map3d map_3d;
  map_2d.setResolutions(Map2d::WorldCoordinate(2.5, 0.5));
  map_3d.setResolutions(Map3d::WorldCoordinate(2.5, 0.5, 1.5));

  EXPECT_EQ(Map2d::MapCoordinate(0, 6), map_2d.worldToMap(Map2d::WorldCoordinate(2, 3)));
  EXPECT_EQ(Map3d::MapCoordinate(0, 6, 2), map_3d.worldToMap(Map3d::WorldCoordinate(2, 3, 4)));

  EXPECT_EQ(Map2d::WorldCoordinate(12.5, 0.5), map_2d.mapToWorld(Map2d::MapCoordinate(5, 1)));
  EXPECT_EQ(Map3d::WorldCoordinate(12.5, 0.5, 9), map_3d.mapToWorld(Map3d::MapCoordinate(5, 1, 6)));
}

TEST(ScaledMapLogic, transformations)
{
  using Map2d = ScaledMapLogic<2>;
  using Map3d = ScaledMapLogic<3>;
  Map2d map_2d;
  Map3d map_3d;
  map_2d.reconfigureFromMapSize(Map2d::MapCoordinate{ 4, 8 }, Map2d::WorldCoordinate{ 10, 20 },
                                Map2d::WorldCoordinate{ 2.5, 0.5 });
  map_3d.reconfigureFromMapSize(Map3d::MapCoordinate{ 4, 8, 10 }, Map3d::WorldCoordinate{ 10, 20, 30 },
                                Map3d::WorldCoordinate{ 2.5, 0.5, 1.5 });

  EXPECT_EQ(Map2d::MapCoordinate(-3, -34), map_2d.worldToMap(Map2d::WorldCoordinate(2, 3)));
  EXPECT_EQ(Map3d::MapCoordinate(-3, -34, -17), map_3d.worldToMap(Map3d::WorldCoordinate(2, 3, 4)));

  EXPECT_EQ(Map2d::WorldCoordinate(22.5, 20.5), map_2d.mapToWorld(Map2d::MapCoordinate(5, 1)));
  EXPECT_EQ(Map3d::WorldCoordinate(22.5, 20.5, 39), map_3d.mapToWorld(Map3d::MapCoordinate(5, 1, 6)));
}

TEST(ScaledMapLogic, inside_map)
{
  using Map2d = ScaledMapLogic<2>;
  using Map3d = ScaledMapLogic<3>;
  Map2d map_2d;
  Map3d map_3d;
  map_2d.reconfigureFromMapSize(Map2d::MapCoordinate{ 4, 8 }, Map2d::WorldCoordinate{ 0, 0 },
                                Map2d::WorldCoordinate{ 1, 1 });
  map_3d.reconfigureFromMapSize(Map3d::MapCoordinate{ 4, 8, 10 }, Map3d::WorldCoordinate{ 0, 0, 0 },
                                Map3d::WorldCoordinate{ 1, 1, 1 });

  EXPECT_TRUE(map_2d.isInMap({ 0, 0 }));
  EXPECT_TRUE(map_3d.isInMap({ 0, 0, 0 }));

  EXPECT_FALSE(map_2d.isInMap({ -1, 0 }));
  EXPECT_FALSE(map_3d.isInMap({ 0, -1, 0 }));

  EXPECT_TRUE(map_2d.isInMap({ 3, 7 }));
  EXPECT_TRUE(map_3d.isInMap({ 3, 7, 9 }));

  EXPECT_FALSE(map_2d.isInMap({ 3, 8 }));
  EXPECT_FALSE(map_3d.isInMap({ 3, 8, 10 }));

  EXPECT_FALSE(map_2d.isInMap({ 4, 8 }));
  EXPECT_FALSE(map_3d.isInMap({ 4, 8, 10 }));

  EXPECT_FALSE(map_2d.isInMap({ 3, 9 }));
  EXPECT_FALSE(map_3d.isInMap({ 3, 7, 11 }));
}
