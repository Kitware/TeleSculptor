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
::camera_intrinsics_(const Eigen::Matrix<T,3,3>& K)
: focal_length_(K(0,0)),
  principal_point_(K(0,2), K(1,2)),
  aspect_ratio_(K(0,0)/K(1,1))
{
}


/// Convert to a 3x3 calibration matrix
template <typename T>
camera_intrinsics_<T>
::operator Eigen::Matrix<T,3,3>() const
{
  Eigen::Matrix<T,3,3> K;
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
template class MAPTK_LIB_EXPORT camera_intrinsics_<T>; \
template MAPTK_LIB_EXPORT std::ostream& operator<<(std::ostream& s, const camera_intrinsics_<T>& k); \
template MAPTK_LIB_EXPORT std::istream& operator>>(std::istream& s, camera_intrinsics_<T>& k)

INSTANTIATE_CAMERA_INTRINSICS(double);
INSTANTIATE_CAMERA_INTRINSICS(float);

#undef INSTANTIATE_CAMERA_INTRINSICS
/// \endcond

} // end namespace maptk
