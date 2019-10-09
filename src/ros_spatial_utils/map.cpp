// ros_spatial_utils - A library of spatial utilities
// Copyright (C) 2019  Arvid Norlander
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.
#include "ros_spatial_utils/map.h"

#include "ros_spatial_utils/distance_calculator.h"

#include <Eigen/Core>

//! @file
//! @brief Defines a map that can be loaded from a nav_msgs/OccupancyGrid.

namespace ros_spatial_utils
{
Map::Map()
{
}

void Map::newMap(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  // Store a copy of the metadata, we will need stuff from it.
  metadata_ = msg->info;

  // Note: We assume that there is no rotation here, otherwise it will be
  // annoying.
  reconfigureFromMapSize(MapCoordinate{ metadata_.width, metadata_.height },
                         WorldCoordinate{ metadata_.origin.position.x, metadata_.origin.position.y },
                         metadata_.resolution * WorldCoordinate::Ones());

  // Compute free space map.
  computeStateMap(*msg, &map_state_);

  // Pre-compute a vector of free spaces to allow uniform sampling from it.
  free_cells_.clear();
  for (Eigen::Index y = 0; y < map_state_.rows(); y++)
  {
    for (Eigen::Index x = 0; x < map_state_.cols(); x++)
    {
      if (map_state_(y, x) == MapState::Free)
      {
        free_cells_.push_back(MapCoordinate(x, y));
      }
    }
  }
  free_cells_dist_ = std::uniform_int_distribution<size_t>(0, free_cells_.size() - 1);
}

Pose2D<Map::WorldScalar> Map::generateRandomPose() const
{
  WorldCoordinate free_cell = mapToWorld(free_cells_.at(free_cells_dist_(rng_)));

  return Pose2D<Map::WorldScalar>(free_cell(0), free_cell(1), rotation_dist_(rng_));
}

MapState Map::getMapStateAt(const WorldCoordinate& world_coord) const
{
  // Convert to grid coordinates
  MapCoordinate actual_pos = worldToMap(world_coord);

  // Penalise being outside map.
  MapState p = MapState::Unknown;
  if (isInMap(actual_pos))
  {
    p = map_state_(actual_pos(1), actual_pos(0));
  }
  return p;
}

void computeStateMap(const nav_msgs::OccupancyGrid& map, MapStateContainer* map_state)
{
  auto height = map.info.height;
  auto width = map.info.width;
  map_state->resize(height, width);

  for (Eigen::Index row = 0; row < map_state->rows(); row++)
  {
    for (Eigen::Index col = 0; col < map_state->cols(); col++)
    {
      auto value = map.data[static_cast<size_t>(row * width + col)];
      if (value == -1)
      {
        // Unknown
        (*map_state)(row, col) = MapState::Unknown;
      }
      else if (value < 35)
      {
        // Free space
        (*map_state)(row, col) = MapState::Free;
      }
      else if (value > 65)
      {
        // Occupied
        (*map_state)(row, col) = MapState::Occupied;
      }
      else
      {
        // Unsure
        (*map_state)(row, col) = MapState::Unknown;
      }
    }
  }
}

}  // namespace ros_spatial_utils
