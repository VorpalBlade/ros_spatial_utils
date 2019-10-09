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
#include "ros_spatial_utils/distance_calculator.h"

#include <cmath>
#include <memory>
#include <queue>

//! @file
//! @brief Distance calculations in 2D grid maps.

namespace ros_spatial_utils
{
namespace
{
//! Distance cache + metadata to track if the cache is valid.
struct DistanceCacheWithMetadata
{
  //! Actual cache
  DistanceCache cache_;
  //! Resolution of cache
  float resolution_;
  //! Max distance the cache is valid for.
  float max_dist_;
};

//! @brief Cache for the cache
//!
//! Reuses the computed distance cache if it is the same as the previous time.
//!
//! NOT thread-safe!
const DistanceCache& cacheDistanceCache(float resolution, float max_dist)
{
  static std::shared_ptr<DistanceCacheWithMetadata> cache;

  // Exact comparisons of floats is ok in this case, ignore any warnings from
  // the compiler.
  if (!cache || cache->resolution_ != resolution || cache->max_dist_ >= max_dist)
  {
    cache.reset(new DistanceCacheWithMetadata{ createDistanceCache(resolution, max_dist), resolution, max_dist });
  }
  return cache->cache_;
}

//! @brief Helper struct for use with priority queue
struct CellInfo
{
  //! Pointer to the map object containing this
  const MapContainer* source_map_;
  //! @name Coordinates of source obstacle
  //! @{
  Eigen::Index source_x_;
  Eigen::Index source_y_;
  //! @}

  //! @name Coordinates of this cell.
  //! @{
  Eigen::Index x_;
  Eigen::Index y_;
  //! @}

  //! Comparison operator for use with priority queue.
  bool operator<(const CellInfo& other) const
  {
    // Only well defined for same map anyway
    return (*source_map_)(y_, x_) > (*source_map_)(other.y_, other.x_);
  }
};

}  // anonymous namespace

DistanceCache createDistanceCache(float resolution, float max_dist)
{
  // Size in cells
  auto size = Eigen::Index(max_dist / resolution);
  // Add +2 to be able to easily detect when we go outside the the range of
  // interest in process_new_cell()
  Eigen::MatrixXf result(size + 2, size + 2);

  // Fill cache with Euclidean distances
  for (Eigen::Index col = 0; col < result.cols(); col++)
  {
    for (Eigen::Index row = 0; row < result.rows(); row++)
    {
      result(row, col) = std::sqrt(static_cast<float>(col * col + row * row));
    }
  }
  return result;
}

void computeDistanceMap(float max_dist, const nav_msgs::OccupancyGrid& map, MapContainer* output)
{
  auto height = map.info.height;
  auto width = map.info.width;
  auto resolution = map.info.resolution;
  const DistanceCache& distances = cacheDistanceCache(resolution, max_dist);
  // Keeps track of visited cells.
  Eigen::Array<bool, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> visited(height, width);

  output->resize(height, width);

  std::priority_queue<CellInfo> cell_queue;

  // Copy all obstacles across, set them to 1, set every other cell to max_dist
  for (Eigen::Index row = 0; row < output->rows(); row++)
  {
    for (Eigen::Index col = 0; col < output->cols(); col++)
    {
      auto value = map.data[static_cast<size_t>(row * width + col)];
      visited(row, col) = false;
      if (value == -1)
      {
        // Unknown
        (*output)(row, col) = max_dist;
      }
      else if (value < 35)
      {
        // Free space
        (*output)(row, col) = max_dist;
      }
      else if (value > 65)
      {
        // Occupied
        (*output)(row, col) = 0;
        cell_queue.push(CellInfo{ output, col, row, col, row });
        visited(row, col) = true;
      }
      else
      {
        // Unsure
        (*output)(row, col) = max_dist;
      }
    }
  }

  //! Define this as a lambda to capture some parameters
  auto process_new_cell = [&max_dist, &distances, &cell_queue, &visited, &output,
                           &resolution](const CellInfo& parent, Eigen::Index x, Eigen::Index y) -> void {
    if (visited(y, x))
    {
      return;
    }
    visited(y, x) = true;
    auto dist_x = std::abs(parent.source_x_ - x);
    auto dist_y = std::abs(parent.source_y_ - y);
    auto distance = distances(dist_y, dist_x) * resolution;
    if (distance > max_dist)
    {
      return;
    }

    (*output)(y, x) = distance;
    CellInfo child = parent;
    child.x_ = x;
    child.y_ = y;
    cell_queue.push(child);
  };

  // Now process the queue
  while (!cell_queue.empty())
  {
    auto cell_info = cell_queue.top();
    cell_queue.pop();
    // Enqueue neighbours as long as they are inside the borders of the map
    if (cell_info.x_ > 0)
    {
      process_new_cell(cell_info, cell_info.x_ - 1, cell_info.y_);
    }
    if (cell_info.x_ < output->cols() - 1)
    {
      process_new_cell(cell_info, cell_info.x_ + 1, cell_info.y_);
    }
    if (cell_info.y_ > 0)
    {
      process_new_cell(cell_info, cell_info.x_, cell_info.y_ - 1);
    }
    if (cell_info.y_ < output->rows() - 1)
    {
      process_new_cell(cell_info, cell_info.x_, cell_info.y_ + 1);
    }
  }
}

}  // namespace ros_spatial_utils
