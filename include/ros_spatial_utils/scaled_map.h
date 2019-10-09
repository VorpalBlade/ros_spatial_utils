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

#include <Eigen/Core>

//! @file
//! @brief Classes for generic axis aligned maps with a resolution and offset in
//!        the world.

namespace ros_spatial_utils
{
//! @brief Generic scaled infinite map (scaling logic only, no map data).
template <int Dimensions>
class ScaledInfiniteMapLogic
{
public:
  //! Type of coordinate in the world
  using WorldCoordinate = Eigen::Matrix<float, Dimensions, 1>;
  //! Type of coordinate in the map.
  using MapCoordinate = Eigen::Matrix<int, Dimensions, 1>;

  //! Type of scalar in the world coordinate
  using WorldScalar = typename WorldCoordinate::Scalar;
  //! Type of scalar in the map coordinate
  using MapScalar = typename MapCoordinate::Scalar;

  ScaledInfiniteMapLogic() = default;

  //! Compute map coordinates for a given world coordinate.
  inline MapCoordinate worldToMap(const WorldCoordinate& pos) const
  {
    return (pos.array() / resolutions_.array()).matrix().template cast<MapScalar>();
  }

  //! Compute map coordinates for a given world coordinate.
  inline WorldCoordinate mapToWorld(const MapCoordinate& map_pos) const
  {
    return (map_pos.template cast<WorldScalar>().array() * resolutions_.array()).matrix();
  }

  //! @brief Updates the resolution. It is up to the user to ensure the data is
  //!        rescaled for this!
  void setResolutions(const WorldCoordinate& resolutions)
  {
    this->resolutions_ = resolutions;
  }

  //! Gets the current resolutions [m / cell] per dimension.
  const WorldCoordinate& getResolutions() const
  {
    return resolutions_;
  }

  // Fix potential assert
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:
  //! Resolution of map [m / cell], per dimension
  WorldCoordinate resolutions_;
};

//! @brief Generic scaled offset axis-aligned map (scaling logic only, no map
//! data).
template <int Dimensions>
class ScaledMapLogic : public ScaledInfiniteMapLogic<Dimensions>
{
public:
  //! Type of base class.
  using BaseT = ScaledInfiniteMapLogic<Dimensions>;

  ScaledMapLogic() = default;

  //! Check if this is a valid coordinate inside the map.
  inline bool isInMap(const typename BaseT::MapCoordinate& p) const
  {
    return (p.Zero().array() <= p.array()).all() && (p.array() < map_size_.array()).all();
  }

  //! Compute map coordinates for a given world coordinate.
  inline typename BaseT::MapCoordinate worldToMap(const typename BaseT::WorldCoordinate& pos) const
  {
    return BaseT::worldToMap(pos - map_offset_);
  }

  //! Compute map coordinates for a given world coordinate.
  inline typename BaseT::WorldCoordinate mapToWorld(const typename BaseT::MapCoordinate& map_pos) const
  {
    return BaseT::mapToWorld(map_pos) + map_offset_;
  }

  //! Reconfigures the map. Does not ensure consistent data state.
  void reconfigureFromWorldSize(const typename BaseT::WorldCoordinate& world_size,
                                const typename BaseT::WorldCoordinate& world_offset,
                                const typename BaseT::WorldCoordinate& resolutions)
  {
    this->setResolutions(resolutions);
    this->world_size_ = world_size;
    this->map_offset_ = world_offset;
    this->resolutions_ = resolutions;
    typename BaseT::WorldCoordinate world_map_size = (world_size.array() / resolutions.array()).matrix();
    map_size_ = world_map_size.template cast<typename BaseT::MapCoordinate::Scalar>();
  }

  //! Reconfigures the map. Does not ensure consistent data state.
  void reconfigureFromMapSize(const typename BaseT::MapCoordinate& map_size,
                              const typename BaseT::WorldCoordinate& world_offset,
                              const typename BaseT::WorldCoordinate& resolutions)
  {
    this->setResolutions(resolutions);
    this->map_size_ = map_size;
    this->map_offset_ = world_offset;
    this->resolutions_ = resolutions;
    this->world_size_ =
        (map_size.template cast<typename BaseT::WorldCoordinate::Scalar>().array() * resolutions.array()).matrix();
  }

  // Fix potential assert
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:
  //! Size of map in world coordinates
  typename BaseT::WorldCoordinate world_size_;
  //! Offset of (0,0) corner of map in world coordinates
  typename BaseT::WorldCoordinate map_offset_;
  //! Size of map in map coordinates
  typename BaseT::MapCoordinate map_size_;
};

}  // namespace ros_spatial_utils
