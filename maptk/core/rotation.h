/*ckwg +29
 * Copyright 2013-2014 by Kitware, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither name of Kitware, Inc. nor the names of any contributors may be used
 *    to endorse or promote products derived from this software without specific
 *    prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHORS OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * \file
 * \brief Header for \link maptk::rotation_ rotation_<T> \endlink class
 */

#ifndef MAPTK_ROTATION_H_
#define MAPTK_ROTATION_H_

#include <iostream>
#include <vector>

#include "matrix.h"
#include "vector.h"


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
   * \param rot orthonormal matrix to construct from
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


// Generate an interpolated rotation between \c A and \c B by a given fraction
/**
 * \c f must be 0 < \c f < 1
 *
 * TODO: Have this raise an exception when f is not within the valid range.
 *
 * \param A Rotation we are interpolating from.
 * \param B Rotation we are interpolation towards.
 * \param f Fractional value describing the
 * \returns A rotation in between A and B to a degree proportional to the given
 *          fraction.
 */
template <typename T>
MAPTK_CORE_EXPORT
rotation_<T>
interpolate_rotation(rotation_<T> const& A, rotation_<T> const& B, T f);


/// Generate N evenly interpolated rotations inbetween \c A and \c B.
/**
 * \c n must be >= 1.
 *
 * \param[in]   A           Rotation we are interpolating from.
 * \param[in]   B           Rotation we are interpolating towards,
 * \paran[in]   n           Number of even interpolations in between A and B to generate.
 * \param[out]  interp_rots Interpolated rotations are added to this vector in
 *                          in order of generation in the A -> B direction.
 *
 * \returns A vector of \c n evenly interpolated rotations in order between A
 *          and B.
 */
template <typename T>
MAPTK_CORE_EXPORT
void
interpolated_rotations(rotation_<T> const& A, rotation_<T> const& B, size_t n, std::vector< rotation_<T> > & interp_rots);


} // end namespace maptk


#endif // MAPTK_ROTATION_H_
