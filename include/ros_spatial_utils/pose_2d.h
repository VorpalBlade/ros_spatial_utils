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

#include "ros_spatial_utils/angles.h"

#include <Eigen/Geometry>
#include <geometry_msgs/Pose.h>
#include <iosfwd>
#include <tf2/utils.h>

//! @file
//! @brief Pose 2D and related operators

namespace ros_spatial_utils
{
//! @brief 2D pose structure
//!
//! @tparam T Floating point type to use
template <typename T = float>
struct Pose2D
{
  //! Type of member
  using Scalar = T;
  //! Matching eigen vector type
  using EigenVector = Eigen::Matrix<Scalar, 3, 1>;
  //! Matching eigen matrix type
  using EigenMatrix = Eigen::Matrix<Scalar, 3, 3>;

  //! Matching eigen vector type for position part
  using EigenPosition = Eigen::Matrix<Scalar, 2, 1>;

  //! @name Position
  //! @{
  T x_;
  T y_;
  //! @}

  //! Angle in radians [-pi, pi]
  T theta_;

  //! Default constructor
  Pose2D()
  {
    // Can't use "= default" due to
    // https://gcc.gnu.org/bugzilla/show_bug.cgi?id=57728
    // (No idea why it works for the copy constructor below though...)
  }

  //! Value constructor
  Pose2D(T x, T y, T theta) : x_(x), y_(y), theta_(theta)
  {
  }

  //! Copy constructor
  Pose2D(const Pose2D<T>& other) = default;

  //! Default copy assignment
  Pose2D& operator=(const Pose2D<T>& other) = default;

  //! Conversion constructor
  template <typename U>
  inline explicit Pose2D(const Pose2D<U>& other) : x_(other.x_), y_(other.y_), theta_(other.theta_)
  {
  }

  //! Constructor converting from a ROS pose message.
  inline explicit Pose2D(const geometry_msgs::Pose& p)
    : x_(T(p.position.x)), y_(T(p.position.y)), theta_(T(tf2::getYaw(p.orientation)))
  {
  }

  //! Normalise the angle to [-pi, pi]
  void normalise()
  {
    theta_ = normaliseAngle(theta_);
  }

  //! @brief Operator to convert to pose
  //!
  //! Note! Only sensible with normalised angle.
  inline explicit operator geometry_msgs::Pose() const
  {
    geometry_msgs::Pose pose;
    pose.position.x = x_;
    pose.position.y = y_;
    pose.orientation.z = std::sin(theta_ / 2.0f);
    pose.orientation.w = std::cos(theta_ / 2.0f);
    return pose;
  }

  //! Operator to convert the pose to an Eigen 3D transform
  template<typename ScalarT, int Options>
  inline explicit operator Eigen::Transform<ScalarT,3,Options>() const
  {
    using TransformT = Eigen::Transform<ScalarT,3,Options>;
    TransformT transform = TransformT::Identity();
    transform.translate(Eigen::Vector3d(x_, y_, 0));
    transform.rotate(Eigen::AngleAxisd(theta_, Eigen::Vector3d(0, 0, 1)));
    return transform;
  }

  //! Operator to convert the pose to an Eigen 2D transform
  template<typename ScalarT, int Options>
  inline explicit operator Eigen::Transform<ScalarT,2,Options>() const
  {
    using TransformT = Eigen::Transform<ScalarT,2,Options>;
    TransformT transform = TransformT::Identity();
    transform.translate(Eigen::Vector2f(x_, y_));
    transform.rotate(Eigen::Rotation2Df(theta_));
    return transform;
  }

  //! Return just the position part.
  EigenPosition asPosition() const
  {
    return EigenPosition(x_, y_);
  }

  //! @brief Convert to 4D vector.
  //!
  //! Useful for statistics, since this makes it possible to compute the
  //! mean in theta as well.
  //!
  //! @return Vector of x,y,sin(theta),cos(theta)
  inline Eigen::Vector4d toDecomposed() const
  {
    // We cannot sum angles and compute the mean like normal. There are multiple
    // possible definitions of a mean over a cyclic set, but a sensible one is
    // to sum the cos and sin separately (summing the vectors on the unit
    // circle), then taking the angle of that.
    //
    // See also
    // https://stackoverflow.com/questions/491738/how-do-you-calculate-the-average-of-a-set-of-circular-data
    // for further discussion on this.
    return Eigen::Vector4d{
      x_,
      y_,
      std::sin(static_cast<double>(theta_)),
      std::cos(static_cast<double>(theta_)),
    };
  }

  //! @name Pose2D operator overloads
  //!
  //! @brief Various useful operator overloads.
  //!
  //! Note that none of these normalise the angle, since it isn't needed except
  //! at the very end of the computation.
  //! @{
  inline Pose2D<T>& operator+=(const Pose2D<T>& b)
  {
    x_ += b.x_;
    y_ += b.y_;
    theta_ += b.theta_;
    return *this;
  }

  inline Pose2D<T> operator+(const Pose2D<T>& b) const
  {
    Pose2D<T> n(*this);
    n += b;
    return n;
  }

  inline Pose2D<T>& operator-=(const Pose2D<T>& b)
  {
    x_ -= b.x_;
    y_ -= b.y_;
    theta_ -= b.theta_;
    return *this;
  }

  inline Pose2D<T> operator-(const Pose2D<T>& b) const
  {
    Pose2D n(*this);
    n -= b;
    return n;
  }

  inline Pose2D<T>& operator*=(T v)
  {
    x_ *= v;
    y_ *= v;
    theta_ *= v;
    return *this;
  }

  inline Pose2D<T> operator*(T v) const
  {
    Pose2D<T> b(*this);
    b *= v;
    return b;
  }
  //! @}
};

//! @brief Operator overload for scalar multiplication
//!
//! Note that angle will not be normalised.
template <typename T>
Pose2D<T> operator*(T v, const Pose2D<T>& a)
{
  return a * v;
}

//! Stream output operator for logging and debugging
std::ostream& operator<<(std::ostream& os, const Pose2D<float>& pose);

//! Stream output operator for logging and debugging
std::ostream& operator<<(std::ostream& os, const Pose2D<double>& pose);

// Explicit instantiation in a single translation unit.
#ifdef POSE2D_CPP
template struct Pose2D<float>;
template struct Pose2D<double>;
#else
extern template struct Pose2D<float>;
extern template struct Pose2D<double>;
#endif
}  // namespace ros_spatial_utils
