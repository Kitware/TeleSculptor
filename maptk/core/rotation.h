/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_ROTATION_H_
#define MAPTK_ROTATION_H_

#include <iostream>

#include "matrix.h"
#include "vector.h"


/**
 * \file
 * \brief Header for \link maptk::rotation_ rotation_<T> \endlink class
 */


namespace maptk
{

/// A representation of 3D rotation.
/**
 * Internally, rotation is stored in quaternion form
 */
template <typename T>
class MAPTK_CORE_EXPORT rotation_
{
public:
  /// Default Constructor
  rotation_<T>() : q_(0,0,0,1) {}

  /// Copy Constructor from another type
  template <typename U>
  explicit rotation_<T>(const rotation_<U>& other)
  : q_(static_cast<vector_4_<T> >(other.quaternion())) {}

  /// Constructor - from a 4D quaternion vector (i,j,k,r)
  explicit rotation_<T>(const vector_<4,T>& quaternion)
  : q_(quaternion) {}

  /// Constructor - from a Rodrigues vector
  /**
   * A Rodrigues vector is a minimal parameterization of rotation where
   * the direction of the vector is the axis of rotation and the
   * magnitude of the vector is the angle of rotation (in radians).
   * This representation is closely related to the tangent space on
   * the manifold of the group of rotations.
   * \param rvec Rodrigues vector to construct from.
   */
  explicit rotation_<T>(const vector_<3,T>& rvec);

  /// Constructor - from rotation angle and axis
  rotation_<T>(T angle, const vector_<3,T>& axis);

  /// Constructor - from yaw, pitch, and roll
  rotation_<T>(const T& yaw, const T& pitch, const T& roll);

  /// Constructor - from a matrix
  /**
   * requires orthonormal matrix with +1 determinant
   * \param rot orthonormal mactrix to construct from
   */
  explicit rotation_<T>(const matrix_<3,3,T>& rot);

  /// Convert to a 3x3 matrix
  operator matrix_<3,3,T>() const;

  /// Returns the axis of rotation
  /**
   * \note axis is undefined for the identity rotation,
   *       returns (0,0,1) in this case.
   * \sa angle()
   */
  vector_3_<T> axis() const;

  /// Returns the angle of the rotation in radians about the axis
  /**
   * \sa axis()
   */
  T angle() const;

  /// Access the quaternion as a 4-vector
  /**
   * The first 3 components are imaginary (i,j,k) the last is real
   */
  const vector_4_<T>& quaternion() const { return q_; }

  /// Return the rotation as a Rodrigues vector
  vector_3_<T> rodrigues() const;

  /// Convert to yaw, pitch, and roll
  void get_yaw_pitch_roll(T& yaw, T& pitch, T& roll) const;

  /// Compute the inverse rotation
  rotation_<T> inverse() const
  {
    return rotation_<T>(vector_4_<T>(-q_.x(), -q_.y(), -q_.z(), q_.w()));
  }

  /// Compose two rotations
  rotation_<T> operator*(const rotation_<T>& rhs) const;

  /// Rotate a vector
  /**
   * \note for a large number of vectors, it is more efficient to
   *       create a rotation matrix and use matrix multiplcation
   * \param rhs right-hand side vector to operate against
   */
  vector_<3,T> operator*(const vector_<3,T>& rhs) const;

  /// Equality operator
  inline bool operator==(const rotation_<T>& rhs) const
  {
    return this->q_ == rhs.q_;
  }

  /// Inequality operator
  inline bool operator!=(const rotation_<T>& rhs) const
  {
    return !(*this == rhs);
  }

protected:
  /// rotatation stored internally as a quaternion vector
  vector_4_<T> q_;
};


/// Double-precision rotation_ type
typedef rotation_<double> rotation_d;
/// Single-precision rotation_ type
typedef rotation_<float> rotation_f;


/// output stream operator for a rotation
template <typename T>
MAPTK_CORE_EXPORT std::ostream&  operator<<(std::ostream& s, const rotation_<T>& r);

/// input stream operator for a rotation
template <typename T>
MAPTK_CORE_EXPORT std::istream&  operator>>(std::istream& s, rotation_<T>& r);


} // end namespace maptk


#endif // MAPTK_ROTATION_H_
