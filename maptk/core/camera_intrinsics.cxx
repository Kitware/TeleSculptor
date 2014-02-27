/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief Implementation of \link maptk::camera_intrinsics_
 *        camera_intrinsics_<T> \endlink class
 *        for \c T = { \c float, \c double }
 */

#include "camera_intrinsics.h"


namespace maptk
{


/// Constructor - from a calibration matrix
template <typename T>
camera_intrinsics_<T>
::camera_intrinsics_(const matrix_<3,3,T>& K)
: focal_length_(K(0,0)),
  principal_point_(K(0,2), K(1,2)),
  aspect_ratio_(K(0,0)/K(1,1))
{
}


/// Convert to a 3x3 calibration matrix
template <typename T>
camera_intrinsics_<T>
::operator matrix_<3,3,T>() const
{
  matrix_<3,3,T> K;
  K(0,0) = focal_length_;
  K(0,1) = skew_;
  K(0,2) = principal_point_.x();
  K(1,1) = focal_length_ / aspect_ratio_;
  K(1,2) = principal_point_.y();
  K(2,2) = T(1);
  K(1,0) = K(2,0) = K(2,1) = T(0);
  return K;
}


/// Map normalized image coordinates into actual image coordinates
template <typename T>
vector_2_<T>
camera_intrinsics_<T>
::map(const vector_2_<T>& pt) const
{
  const vector_2_<T>& pp = principal_point_;
  return vector_2_<T>(pt.x() * focal_length_ + pt.y() * skew_ + pp.x(),
                      pt.y() * focal_length_ / aspect_ratio_ + pp.y());
}


/// Map a 3D point in camera coordinates into actual image coordinates
template <typename T>
vector_2_<T>
camera_intrinsics_<T>
::map(const vector_3_<T>& norm_hpt) const
{
  return this->map(vector_2_<T>(norm_hpt.x()/norm_hpt.z(),
                                norm_hpt.y()/norm_hpt.z()));
}


/// Unmap actual image coordinates back into normalized image coordinates
template <typename T>
vector_2_<T>
camera_intrinsics_<T>
::unmap(const vector_2_<T>& pt) const
{
  vector_2_<T> p0 = pt - principal_point_;
  T y = p0.y() * aspect_ratio_ / focal_length_;
  return vector_2_<T>((p0.x() - y * skew_) / focal_length_, y);
}


/// output stream operator for a camera intrinsics
template <typename T>
std::ostream&  operator<<(std::ostream& s, const camera_intrinsics_<T>& k)
{
  // TODO: implement me
  return s;
}


/// input stream operator for a camera intrinsics
template <typename T>
std::istream&  operator>>(std::istream& s, camera_intrinsics_<T>& k)
{
  // TODO: implement me
  return s;
}


/// \cond DoxygenSuppress
#define INSTANTIATE_CAMERA_INTRINSICS(T) \
template class MAPTK_CORE_EXPORT camera_intrinsics_<T>; \
template MAPTK_CORE_EXPORT std::ostream& operator<<(std::ostream& s, const camera_intrinsics_<T>& k); \
template MAPTK_CORE_EXPORT std::istream& operator>>(std::istream& s, camera_intrinsics_<T>& k)

INSTANTIATE_CAMERA_INTRINSICS(double);
INSTANTIATE_CAMERA_INTRINSICS(float);

#undef INSTANTIATE_CAMERA_INTRINSICS
/// \endcond

} // end namespace maptk
