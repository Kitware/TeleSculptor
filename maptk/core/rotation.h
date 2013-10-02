/*ckwg +5
 * Copyright 2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_ROTATION_H_
#define MAPTK_ROTATION_H_

#include <iostream>
#include "vector.h"
#include "matrix.h"

namespace maptk
{

/// A representation of 3D rotation.
/// Internally rotation is stored in quaternion form
template <typename T>
class rotation_
{
public:
  /// Default Constructor
  rotation_<T>() : q_(0,0,0,1) {}

  /// Constructor - from a 4D quaternion vector (i,j,k,r)
  rotation_<T>(const vector_<4,T>& quaternion)
  : q_(quaternion) {}

  /// Constructor - from yaw, pitch, and roll
  rotation_<T>(const T& yaw, const T& pitch, const T& roll);

  /// Constructor - from a matrix
  /// requires orthonormal matrix with +1 determinant
  explicit rotation_<T>(const matrix_<3,3,T>& mat);

  /// Convert to a 3x3 matrix
  operator matrix_<3,3,T>() const;

  /// Convert to yaw, pitch, and roll
  void get_yaw_pitch_roll(T& yaw, T& pitch, T& roll);

  /// Compute the inverse rotation
  rotation_<T> inverse() const
  {
    return rotation_<T>(vector_4_<T>(-q_.x(), -q_.y(), -q_.z(), q_.w()));
  }

  /// Compose two rotations
  rotation_<T> operator*(const rotation_<T>& rhs) const;

  /// Rotate a vector
  /// \note for a large number of vectors, it is more efficient to
  /// create a rotation matrix and use matrix multiplcation
  vector_<3,T> operator*(const vector_<3,T>& rhs) const;


protected:
  /// rotatation stored internally as a quaternion vector
  vector_4_<T> q_;
};


typedef rotation_<double> rotation_d;
typedef rotation_<float> rotation_f;


/// output stream operator for a rotation
template <unsigned N, typename T>
std::ostream&  operator<<(std::ostream& s, const rotation_<T>& r);

/// input stream operator for a rotation
template <unsigned N, typename T>
std::istream&  operator>>(std::istream& s, rotation_<T>& r);


} // end namespace maptk


#endif // MAPTK_ROTATION_H_
