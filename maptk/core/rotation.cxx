/*ckwg +5
 * Copyright 2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include "rotation.h"
#include <cmath>

namespace maptk
{


/// Constructor - from yaw, pitch, and roll
template <typename T>
rotation_<T>
::rotation_(const T& yaw, const T& pitch, const T& roll)
{
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
  T x2 = q_[0]*q_[0], xy = q_[0]*q_[1], rx = q_[3]*q_[0],
    y2 = q_[1]*q_[1], yz = q_[1]*q_[2], ry = q_[3]*q_[1],
    z2 = q_[2]*q_[2], zx = q_[2]*q_[0], rz = q_[3]*q_[2],
    r2 = q_[3]*q_[3];
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
template class rotation_<T>; \
template std::ostream&  operator<<(std::ostream& s, const rotation_<T>& r); \
template std::istream&  operator>>(std::istream& s, rotation_<T>& r)

INSTANTIATE_ROTATION(double);
INSTANTIATE_ROTATION(float);

#undef INSTANTIATE_ROTATION
} // end namespace maptk
