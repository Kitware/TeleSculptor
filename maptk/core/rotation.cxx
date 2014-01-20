/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include "rotation.h"

#include <cmath>
#include <limits>
#include <boost/math/constants/constants.hpp>


/**
 * \file
 * \brief Implementation of \link maptk::rotation_ rotation_<T> \endlink
 *        for \c T = { \c float, \c double }
 */

namespace maptk
{


/// Constructor - from yaw, pitch, and roll
template <typename T>
rotation_<T>
::rotation_(const T& yaw, const T& pitch, const T& roll)
{
  using std::sin;
  using std::cos;
  const double pi_over_2 = boost::math::constants::pi<double>() / 2.0;
  const double half_x = 0.5 * static_cast<double>(roll);
  const double half_y = 0.5 * static_cast<double>(pitch);
  const double half_z = 0.5 * static_cast<double>(yaw + pi_over_2);
  rotation_<T> Rx(vector_4_<T>(T(sin(half_x)), 0, 0, T(cos(half_x))));
  rotation_<T> Ry(vector_4_<T>(0, T(sin(half_y)), 0, T(cos(half_y))));
  rotation_<T> Rz(vector_4_<T>(0, 0, T(sin(half_z)), T(cos(half_z))));
  // FIXME this final 180 degree rotation is only needed for CLIF data
  rotation_<T> R180(vector_4_<T>(0, T(sin(pi_over_2)), 0, T(cos(pi_over_2))));
  *this = R180 * Rx * Ry * Rz;
}


/// Constructor - from a matrix
/// requires orthonormal matrix with +1 determinant
template <typename T>
rotation_<T>
::rotation_(const matrix_<3,3,T>& rot)
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
::operator matrix_<3,3,T>() const
{
  T x2 = q_.x()*q_.x(), xy = q_.x()*q_.y(), rx = q_.w()*q_.x(),
    y2 = q_.y()*q_.y(), yz = q_.y()*q_.z(), ry = q_.w()*q_.y(),
    z2 = q_.z()*q_.z(), zx = q_.z()*q_.x(), rz = q_.w()*q_.z(),
    r2 = q_.w()*q_.w();
  matrix_<3,3,T> mat;
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


/// Convert to yaw, pitch, and roll
template <typename T>
void
rotation_<T>
::get_yaw_pitch_roll(T& yaw, T& pitch, T& roll) const
{
  matrix_<3,3,T> rotM(*this);
  T xy = T(std::sqrt(double(rotM(0,0)*rotM(0,0)) + rotM(1,0)*rotM(1,0)));
  if( xy > std::numeric_limits<T>::epsilon() * T(8) )
  {
    yaw   = T(std::atan2(double(rotM(2,1)),double(rotM(2,2))));
    pitch = T(std::atan2(double(-rotM(2,0)),double(xy)));
    roll  = T(std::atan2(double(rotM(1,0)),double(rotM(0,0))));
  }
  else
  {
    yaw   = T(std::atan2(double(-rotM(1,2)),double(rotM(1,1))));
    pitch = T(std::atan2(double(-rotM(2,0)),double(xy)));
    roll  = T(0);
  }
}


/// Compose two rotations
template <typename T>
rotation_<T>
rotation_<T>
::operator*(const rotation_<T>& rhs) const
{
  vector_<4,T> comp_q;
  const vector_<4,T>& q1 = this->q_;
  const vector_<4,T>& q2 = rhs.q_;
  comp_q[3] = q1[3]*q2[3] - q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2];
  comp_q[0] = q1[3]*q2[0] + q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1];
  comp_q[1] = q1[3]*q2[1] + q1[1]*q2[3] + q1[2]*q2[0] - q1[0]*q2[2];
  comp_q[2] = q1[3]*q2[2] + q1[2]*q2[3] + q1[0]*q2[1] - q1[1]*q2[0];
  return rotation_<T>(comp_q);
}


/// Rotate a vector
/// \note for a large number of vectors, it is more efficient to
/// create a rotation matrix and use matrix multiplcation
template <typename T>
vector_<3,T>
rotation_<T>
::operator*(const vector_<3,T>& rhs) const
{
  const T& real = q_.w();
  const vector_3_<T> imag(q_.x(), q_.y(), q_.z());
  const vector_<3,T> ixv(cross_product(imag, rhs));
  return rhs + T(2*real)*ixv - T(2)*cross_product(ixv,imag);
}


/// output stream operator for a rotation
template <typename T>
std::ostream&  operator<<(std::ostream& s, const rotation_<T>& r)
{
  return s;
}


/// input stream operator for a rotation
template <typename T>
std::istream&  operator>>(std::istream& s, rotation_<T>& r)
{
  return s;
}


#define INSTANTIATE_ROTATION(T) \
template class MAPTK_CORE_EXPORT rotation_<T>; \
template MAPTK_CORE_EXPORT std::ostream&  operator<<(std::ostream& s, const rotation_<T>& r); \
template MAPTK_CORE_EXPORT std::istream&  operator>>(std::istream& s, rotation_<T>& r)

INSTANTIATE_ROTATION(double);
INSTANTIATE_ROTATION(float);

#undef INSTANTIATE_ROTATION
} // end namespace maptk
