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

#include <algorithm>
#include <cmath>
#include <type_traits>

//! @file
//! @brief Some angle calculation utility functions

namespace ros_spatial_utils
{
//! @brief Ensure angle in [-pi,pi]
template <typename T>
T constexpr normaliseAngle(T angle)
{
  static_assert(std::is_floating_point<T>::value, "You want a floating point type!");
  // From
  // https://stackoverflow.com/questions/24234609/standard-way-to-normalize-an-angle-to-%CF%80-radians-in-java
  return angle - 2 * M_PI * std::floor((angle + M_PI) / (2 * M_PI));
}

//! @brief Compute smallest signed difference between two angles
template <typename T>
constexpr T angleDelta(T a, T b)
{
  static_assert(std::is_floating_point<T>::value, "You want a floating point type!");
  // Based on
  // https://stackoverflow.com/questions/1878907/the-smallest-difference-between-2-angles
  // but made to return angled result indicating direction by doing quite a bit
  // of working out on paper.
  a = normaliseAngle(a);
  b = normaliseAngle(b);
  auto d1 = a - b;
  auto d2 = 2 * M_PI - std::abs(d1);
  // If d1 is positive, d2 should be negative, since it is going the other way
  // around. Because of the abs() above, the reverse case needs no correction.
  if (0 < d1)
  {
    d2 = -d2;
  }
  // Determine if d1 or d2 should be used based on magnitude
  if (std::abs(d1) < std::abs(d2))
  {
    return d1;
  }
  else
  {
    return d2;
  }
}

//! Convert degrees to radians
template <typename T>
constexpr inline T radians(T v)
{
  static_assert(std::is_floating_point<T>::value, "You want a floating point type!");
  return v * (M_PI / 180);
}

//! Convert radians to degrees
template <typename T>
constexpr inline T degrees(T v)
{
  static_assert(std::is_floating_point<T>::value, "You want a floating point type!");
  return v / (M_PI / 180);
}

}  // namespace ros_spatial_utils
