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
 * \brief Implementation of \link maptk::camera_ camera_<T> \endlink class
 *        for \c T = { \c float, \c double }
 */

#include "camera.h"
#include <iomanip>
#include "transform.h"
#include <typeinfo>
#include "exceptions/base.h"


namespace maptk
{

/// output stream operator for a base class camera
std::ostream& operator<<(std::ostream& s, const camera& c)
{
  using std::setprecision;
  s << setprecision(12) << matrix_3x3d(c.intrinsics()) << "\n"
    << setprecision(12) << matrix_3x3d(c.rotation()) << "\n"
    << setprecision(12) << c.translation() << "\n\n"
    << "0\n";
  return s;
}


/// Rotate the camera about its center such that it looks at the given point.
template <typename T>
void
camera_<T>
::look_at(const Eigen::Matrix<T,3,1>& stare_point,
          const Eigen::Matrix<T,3,1>& up_direction)
{
  // a unit vector in the up direction
  const Eigen::Matrix<T,3,1> up = normalized(up_direction);
  // a unit vector in the look direction (camera Z-axis)
  const Eigen::Matrix<T,3,1> z = normalized(stare_point - get_center());

  // the X-axis of the camera is perpendicular to up and z
  Eigen::Matrix<T,3,1> x = cross_product(-up, z);
  T x_mag = x.magnitude();

  // if the cross product magnitude is small then the up and z vectors are
  // nearly parallel and the up direction is poorly defined.
  if( x_mag < 1e-4)
  {
    std::cerr << "WARNING: camera_::look_at up_direction is "
              << "nearly parallel with the look direction" << std::endl;
  }

  x /= x_mag;
  Eigen::Matrix<T,3,1> y = normalized(cross_product(z,x));

  T r[] = { x.x(), x.y(), x.z(),
            y.x(), y.y(), y.z(),
            z.x(), z.y(), z.z() };

  matrix_<3,3,T> R(r);
  this->set_rotation(rotation_<T>(R));
}


/// Convert to a 3x4 homogeneous projection matrix
template <typename T>
camera_<T>
::operator Eigen::Matrix<T,3,4>() const
{
  Eigen::Matrix<T,3,4> P;
  Eigen::Matrix<T,3,3> R(this->get_rotation());
  Eigen::Matrix<T,3,3> K(this->get_intrinsics());
  Eigen::Matrix<T,3,1> t(this->get_translation());
  P.update(R);
  P.set_column(3,t);
  return K * P;
}


/// Project a 3D point into a 2D image point
template <typename T>
Eigen::Matrix<T,2,1>
camera_<T>
::project(const Eigen::Matrix<T,3,1>& pt) const
{
  return this->intrinsics_.map(this->orientation_ * (pt - this->center_));
}


/// Transform the camera by applying a similarity transformation in place
template <typename T>
camera_<T>&
camera_<T>
::apply_transform(const similarity_<T>& xform)
{
  this->center_ = xform * this->center_;
  this->orientation_ = this->orientation_ * xform.rotation().inverse();
  this->center_covar_ = maptk::transform(this->center_covar_, xform);
  return *this;
}


template <typename T>
std::ostream&  operator<<(std::ostream& s, const camera_<T>& k)
{
  using std::setprecision;
  s << setprecision(12) << matrix_<3,3,T>(k.get_intrinsics()) << "\n"
    << setprecision(12) << matrix_<3,3,T>(k.get_rotation()) << "\n"
    << setprecision(12) << k.get_translation() << "\n\n"
    << "0\n";
  return s;
}


/// input stream operator for a camera intrinsics
template <typename T>
std::istream&  operator>>(std::istream& s, camera_<T>& k)
{
  Eigen<3,3,T> K, R;
  vector_<3,T> t;
  double d;
  s >> K >> R >> t >> d;
  k.set_intrinsics(camera_intrinsics_<T>(K));
  k.set_rotation(rotation_<T>(R));
  k.set_translation(t);
  return s;
}


/// Generate an interpolated camera between \c A and \c B by a given fraction \c f
template <typename T>
camera_<T> interpolate_camera(camera_<T> const& A, camera_<T> const& B, T f)
{
  T f1 = static_cast<T>(1.0) - f;

  // interpolate intrinsics
  camera_intrinsics_<T> k1 = A.get_intrinsics(),
                        k2 = B.get_intrinsics(),
                        k;

  T focal_len = f1*k1.focal_length() + f*k2.focal_length();
  vector_<2,T> principal_point = f1*k1.principal_point() + f*k2.principal_point();
  T aspect_ratio = f1*k1.aspect_ratio() + f*k2.aspect_ratio();
  T skew = f1*k1.skew() + f*k2.skew();
  k = camera_intrinsics_<T>(focal_len, principal_point, aspect_ratio, skew);

  // interpolate center
  Eigen::Matrix<T,3,1> c = f1*A.get_center() + f*B.get_center();

  // interpolate rotation
  rotation_<T> R = interpolate_rotation(A.get_rotation(), B.get_rotation(), f);

  return camera_<T>(c, R, k);
}


/// Generate N evenly interpolated cameras in between \c A and \c B
template <typename T>
void
interpolated_cameras(camera_<T> const& A,
                     camera_<T> const& B,
                     size_t n,
                     std::vector< camera_<T> > & interp_cams)
{
  interp_cams.reserve(interp_cams.capacity() + n);
  size_t denom = n + 1;
  for (size_t i=1; i < denom; ++i)
  {
    interp_cams.push_back(interpolate_camera<T>(A, B, static_cast<T>(i) / denom));
  }
}


/// \cond DoxygenSuppress
#define INSTANTIATE_CAMERA(T) \
template class MAPTK_LIB_EXPORT camera_<T>; \
template MAPTK_LIB_EXPORT std::ostream& operator<<(std::ostream& s, const camera_<T>& c); \
template MAPTK_LIB_EXPORT std::istream& operator>>(std::istream& s, camera_<T>& c); \
template MAPTK_LIB_EXPORT camera_<T> interpolate_camera(camera_<T> const& A, \
                                                         camera_<T> const& B, \
                                                         T f); \
template MAPTK_LIB_EXPORT void interpolated_cameras(camera_<T> const& A, \
                                                     camera_<T> const& B, \
                                                     size_t n, \
                                                     std::vector< camera_<T> > & interp_cams)

INSTANTIATE_CAMERA(double);
INSTANTIATE_CAMERA(float);

#undef INSTANTIATE_CAMERA
/// \endcond


/// Genreate an interpolated camera from sptrs
camera_sptr interpolate_camera(camera_sptr A, camera_sptr B, double f)
{
  double f1 = 1.0 - f;

  // interpolate intrinsics
  camera_intrinsics_d k1 = A->intrinsics(),
                      k2 = B->intrinsics(),
                      k;
  double focal_len = f1*k1.focal_length() + f*k2.focal_length();
  vector_2d principal_point = f1*k1.principal_point() + f*k2.principal_point();
  double aspect_ratio = f1*k1.aspect_ratio() + f*k2.aspect_ratio();
  double skew = f1*k1.skew() + f*k2.skew();
  k = camera_intrinsics_d(focal_len, principal_point, aspect_ratio, skew);

  // interpolate center
  vector_3d c = f1*A->center() + f*B->center();

  // interpolate rotation
  rotation_d R = interpolate_rotation(A->rotation(), B->rotation(), f);

  return camera_sptr(new camera_<double>(c, R, k));
}


} // end namespace maptk
