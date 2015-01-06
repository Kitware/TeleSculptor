/*ckwg +29
 * Copyright 2013-2015 by Kitware, Inc.
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
 * \brief Implementation of \link maptk::rotation_ rotation_<T> \endlink
 *        for \c T = { \c float, \c double }
 */

#include "rotation.h"
#include "eigen_io.h"

#include <cmath>
#include <limits>
#include <boost/math/constants/constants.hpp>


namespace
{

/// helper function to covert axis/angle into quaternion
template <typename T>
maptk::vector_4_<T>
quaternion_from_axis_angle(const Eigen::Matrix<T,3,1>& axis, T angle)
{
  maptk::vector_4_<T> q;
  T a = angle / T(2);
  T sa = std::sin(a);
  q[0] = sa * axis[0];
  q[1] = sa * axis[1];
  q[2] = sa * axis[2];
  q[3] = std::cos(a);
  return q;
}

} // end anonymous namespace


namespace maptk
{

/// Constructor - from a Rodrigues vector
template <typename T>
rotation_<T>
::rotation_(const Eigen::Matrix<T,3,1>& rvec)
{
  T mag = rvec.norm();
  if (mag == T(0))
  {
    // identity rotation is a special case
    q_ = vector_4_<T>(0,0,0,1);
  }
  else
  {
    q_ = quaternion_from_axis_angle((rvec/mag).eval(), mag);
  }
}


/// Constructor - from rotation angle and axis
template <typename T>
rotation_<T>
::rotation_(T angle, const Eigen::Matrix<T,3,1>& axis)
  : q_(quaternion_from_axis_angle(axis.normalized(), angle))
{
}


/// Constructor - from yaw, pitch, and roll
template <typename T>
rotation_<T>
::rotation_(const T& yaw, const T& pitch, const T& roll)
{
  using std::sin;
  using std::cos;
  // compute the rotation from North-East-Down (NED) coordinates to
  // East-North-Up coordinates (ENU). It is a 180 degree rotation about
  // the axis [1/sqrt(2), 1/sqrt(2), 0]
  const double root_two = boost::math::constants::root_two<double>();
  const T inv_root_two = static_cast<T>(1.0/root_two);
  const rotation_<T> Rned2enu(vector_4_<T>(inv_root_two, inv_root_two, 0, 0));
  const double half_x = 0.5 * static_cast<double>(-roll);
  const double half_y = 0.5 * static_cast<double>(-pitch);
  const double half_z = 0.5 * static_cast<double>(-yaw);
  rotation_<T> Rx(vector_4_<T>(T(sin(half_x)), 0, 0, T(cos(half_x))));
  rotation_<T> Ry(vector_4_<T>(0, T(sin(half_y)), 0, T(cos(half_y))));
  rotation_<T> Rz(vector_4_<T>(0, 0, T(sin(half_z)), T(cos(half_z))));
  *this = Rx * Ry * Rz * Rned2enu;
}


/// Constructor - from a matrix
/**
 * requires orthonormal matrix with +1 determinant
 */
template <typename T>
rotation_<T>
::rotation_(const Eigen::Matrix<T,3,3>& rot)
{
  // use double caclulations for more accuracy
  double d0 = rot(0,0), d1 = rot(1,1), d2 = rot(2,2);
  double xx = 1.0 + d0 - d1 - d2;
  double yy = 1.0 - d0 + d1 - d2;
  double zz = 1.0 - d0 - d1 + d2;
  double rr = 1.0 + d0 + d1 + d2;

  // determine the maximum term
  double max_v = rr;
  if (xx > max_v) max_v = xx;
  if (yy > max_v) max_v = yy;
  if (zz > max_v) max_v = zz;

  // choose the derivation that involves division by
  // the largest term, for stability
  if (rr == max_v) {
    T r4 = T(std::sqrt(rr)*2);
    q_[3] = r4 / 4;
    r4 = T(1) / r4;
    q_[0] = (rot(2,1) - rot(1,2)) * r4;
    q_[1] = (rot(0,2) - rot(2,0)) * r4;
    q_[2] = (rot(1,0) - rot(0,1)) * r4;
  }
  else if (xx == max_v) {
    T x4 = T(std::sqrt(xx)*2);
    q_[0] = x4 / 4;
    x4 = T(1) / x4;
    q_[1] = (rot(1,0) + rot(0,1)) * x4;
    q_[2] = (rot(2,0) + rot(0,2)) * x4;
    q_[3] = (rot(2,1) - rot(1,2)) * x4;
  }
  else if (yy == max_v) {
    T y4 = T(std::sqrt(yy)*2);
    q_[1] =  y4 / 4;
    y4 = T(1) / y4;
    q_[0] = (rot(1,0) + rot(0,1)) * y4;
    q_[2] = (rot(2,1) + rot(1,2)) * y4;
    q_[3] = (rot(0,2) - rot(2,0)) * y4;
  }
  else {
    T z4 = T(std::sqrt(zz)*2);
    q_[2] =  z4 / 4;
    z4 = T(1) / z4;
    q_[0] = (rot(2,0) + rot(0,2)) * z4;
    q_[1] = (rot(2,1) + rot(1,2)) * z4;
    q_[3] = (rot(1,0) - rot(0,1)) * z4;
  }
}


/// Convert to a 3x3 matrix
template <typename T>
rotation_<T>
::operator Eigen::Matrix<T,3,3>() const
{
  T x2 = q_.x()*q_.x(), xy = q_.x()*q_.y(), rx = q_.w()*q_.x(),
    y2 = q_.y()*q_.y(), yz = q_.y()*q_.z(), ry = q_.w()*q_.y(),
    z2 = q_.z()*q_.z(), zx = q_.z()*q_.x(), rz = q_.w()*q_.z(),
    r2 = q_.w()*q_.w();
  Eigen::Matrix<T,3,3> mat;
  // fill diagonal terms
  mat(0,0) = r2 + x2 - y2 - z2;
  mat(1,1) = r2 - x2 + y2 - z2;
  mat(2,2) = r2 - x2 - y2 + z2;
  // fill off diagonal terms
  mat(0,1) = 2 * (xy - rz);
  mat(0,2) = 2 * (zx + ry);
  mat(1,2) = 2 * (yz - rx);
  mat(1,0) = 2 * (xy + rz);
  mat(2,0) = 2 * (zx - ry);
  mat(2,1) = 2 * (yz + rx);
  return mat;
}



/// Returns the axis of rotation
template <typename T>
vector_3_<T>
rotation_<T>
::axis() const
{
  vector_3_<T> dir(q_.x(), q_.y(), q_.z());
  T mag = dir.norm();
  if (mag == T(0))
  {
    return vector_3_<T>(0,0,1);
  }
  return dir / mag;
}


/// Returns the angle of the rotation in radians about the axis
template <typename T>
T
rotation_<T>
::angle() const
{
  const double i = vector_3_<T>(q_.x(), q_.y(), q_.z()).norm();
  const double r = q_.w();
  T a = static_cast<T>(2.0 * std::atan2(i, r));
  const T pi = boost::math::constants::pi<T>();
  const T two_pi = static_cast<T>(2) * boost::math::constants::pi<T>();
  // make sure computed angle lies within a sensible range,
  // i.e. -pi/2 < a < pi/2
  if (a >= pi)
  {
    a -= two_pi;
  }
  if (a <= -pi)
  {
    a += two_pi;
  }
  return a;
}


/// Return the rotation as a Rodrigues vector
template <typename T>
vector_3_<T>
rotation_<T>
::rodrigues() const
{
  T angle = this->angle();
  if (angle == 0.0)
  {
    return vector_3_<T>(0,0,0);
  }
  return this->axis() * angle;
}


/// Convert to yaw, pitch, and roll
template <typename T>
void
rotation_<T>
::get_yaw_pitch_roll(T& yaw, T& pitch, T& roll) const
{
  Eigen::Matrix<T,3,3> rotM(*this);
  T cos_p = T(std::sqrt(double(rotM(1,2)*rotM(1,2)) + rotM(2,2)*rotM(2,2)));
  yaw   = T(std::atan2(double(rotM(0,0)),double(rotM(0,1))));
  pitch = T(std::atan2(double(rotM(0,2)),double(cos_p)));
  roll  = T(std::atan2(double(-rotM(1,2)),double(-rotM(2,2))));
}


/// Compose two rotations
template <typename T>
rotation_<T>
rotation_<T>
::operator*(const rotation_<T>& rhs) const
{
  Eigen::Matrix<T,4,1> comp_q;
  const Eigen::Matrix<T,4,1>& q1 = this->q_;
  const Eigen::Matrix<T,4,1>& q2 = rhs.q_;
  comp_q[3] = q1[3]*q2[3] - q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2];
  comp_q[0] = q1[3]*q2[0] + q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1];
  comp_q[1] = q1[3]*q2[1] + q1[1]*q2[3] + q1[2]*q2[0] - q1[0]*q2[2];
  comp_q[2] = q1[3]*q2[2] + q1[2]*q2[3] + q1[0]*q2[1] - q1[1]*q2[0];
  return rotation_<T>(comp_q);
}


/// Rotate a vector
/**
 * \note for a large number of vectors, it is more efficient to
 * create a rotation matrix and use matrix multiplcation
 */
template <typename T>
Eigen::Matrix<T,3,1>
rotation_<T>
::operator*(const Eigen::Matrix<T,3,1>& rhs) const
{
  const T& real = q_.w();
  const vector_3_<T> imag(q_.x(), q_.y(), q_.z());
  const Eigen::Matrix<T,3,1> ixv(imag.cross(rhs));
  return rhs + T(2*real)*ixv - T(2)*ixv.cross(imag);
}


/// output stream operator for a rotation
template <typename T>
std::ostream&  operator<<(std::ostream& s, const rotation_<T>& r)
{
  s << r.quaternion();
  return s;
}


/// input stream operator for a rotation
template <typename T>
std::istream&  operator>>(std::istream& s, rotation_<T>& r)
{
  vector_4_<T> q;
  s >> q;
  r = rotation_<T>(q);
  return s;
}


/// Generate a rotation vector that, when applied to A N times, produces B.
template <typename T>
rotation_<T>
interpolate_rotation(rotation_<T> const& A, rotation_<T> const& B, T f)
{
  // rotation from A -> B
  rotation_<T> C = A.inverse() * B;
  // Reduce the angle of rotation by the fraction provided
  return A * rotation_<T>(C.angle() * f, C.axis());
}


/// Generate N evenly interpolated rotations inbetween \c A and \c B.
template <typename T>
void
interpolated_rotations(rotation_<T> const& A, rotation_<T> const& B, size_t n, std::vector< rotation_<T> > & interp_rots)
{
  interp_rots.reserve(interp_rots.capacity() + n);
  size_t denom = n + 1;
  for (size_t i=1; i<denom; ++i)
  {
    interp_rots.push_back(interpolate_rotation<T>(A, B, static_cast<T>(i) / denom));
  }
}


/// \cond DoxygenSuppress
#define INSTANTIATE_ROTATION(T) \
template class MAPTK_LIB_EXPORT rotation_<T>; \
template MAPTK_LIB_EXPORT std::ostream&  operator<<(std::ostream& s, const rotation_<T>& r); \
template MAPTK_LIB_EXPORT std::istream&  operator>>(std::istream& s, rotation_<T>& r); \
template MAPTK_LIB_EXPORT rotation_<T> interpolate_rotation(rotation_<T> const& A, rotation_<T> const& B, T f); \
template MAPTK_LIB_EXPORT void \
interpolated_rotations(rotation_<T> const& A, rotation_<T> const& B, size_t n, std::vector< rotation_<T> > & interp_rots)

INSTANTIATE_ROTATION(double);
INSTANTIATE_ROTATION(float);

#undef INSTANTIATE_ROTATION
/// \endcond

} // end namespace maptk
