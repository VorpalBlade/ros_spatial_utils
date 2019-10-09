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

#include <Eigen/Core>
#include <nav_msgs/OccupancyGrid.h>

//! @file
//! @brief AMCL style distance map calculations.

namespace ros_spatial_utils {

//! Type of precomputed distance map.
using DistanceCache = Eigen::MatrixXf;

//! @brief Pre-compute a quadrant of Euclidean distances
//! @param resolution  Map resolution as defined by ROS (m / cell)
//! @param max_dist    Maximum distance
//!
//! @return A matrix with distances to (0,0), which is in the upper left corner
//!         of the matrix.
DistanceCache create_distance_cache(float resolution, float max_dist);

//! @brief Calculate a distance-to-obstacle map
//!
//! @param max_dist   Maximum distance from obstacle to track
//! @param map        Input map.
//! @param output     Output map.
//!
//! This algorithm is based on AMCL, but encoded differently due to different
//! map representations. In the output map:
//! * Positive values < max_dist = Distance to nearest obstacle
//! * max_dist = Free space, further away
//! * 0 = Obstacle
void compute_distance_map(float max_dist,
                          const nav_msgs::OccupancyGrid &map,
                          MapContainer *output);

//! @brief Calculate a free/occupied/unknown map
//!
//! @param map        Input map.
//! @param map_state  Output map.
//!
//!
void compute_state_map(const nav_msgs::OccupancyGrid &map,
                       MapStateContainer *map_state);

} // namespace ros_spatial_utils
