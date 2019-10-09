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
#pragma once

#include "ros_spatial_utils/map_types.h"
#include "ros_spatial_utils/pose_2d.h"
#include "ros_spatial_utils/scaled_map.h"

#include <nav_msgs/OccupancyGrid.h>
#include <vector>

//! @file
//! @brief Defines a map that can be loaded from a nav_msgs/OccupancyGrid.

namespace ros_spatial_utils
{
//! Basic map implementation.
class Map : public ScaledMapLogic<2>
{
public:
  //! Constructor
  Map();

  // See Map for documentation.
  void newMap(const nav_msgs::OccupancyGrid::ConstPtr& msg);

  Pose2D<WorldScalar> generateRandomPose() const;

  //! @brief Get the map state at the specified world pose. This takes care of
  //!        offset and resolution.
  MapState getMapStateAt(const WorldCoordinate& world_coord) const;

  // Fix potential assert
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
protected:
  //! Metadata of the most recent map.
  nav_msgs::MapMetaData metadata_;

  //! Map state for each cell.
  MapStateContainer map_state_;

private:
  //! Indices of free cells in map, used for generating random poses
  std::vector<MapCoordinate> free_cells_;
  //! Distribution for free cells.
  mutable std::uniform_int_distribution<size_t> free_cells_dist_;
  //! Distribution for rotation
  mutable std::uniform_real_distribution<float> rotation_dist_ =
      std::uniform_real_distribution<float>(static_cast<float>(-M_PI), static_cast<float>(M_PI));

  //! Random number generator used to generate random valid poses.
  mutable std::mt19937 rng_ = std::mt19937{ std::random_device()() };
};

//! @brief Calculate a free/occupied/unknown map
//!
//! @param map        Input map.
//! @param map_state  Output map.
void computeStateMap(const nav_msgs::OccupancyGrid& map, MapStateContainer* map_state);

}  // namespace ros_spatial_utils
