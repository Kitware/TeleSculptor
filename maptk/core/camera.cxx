/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include "camera.h"
#include <iomanip>


/**
 * \file
 * \brief Implementation of \link maptk::camera_ camera_<T> \endlink class
 *        for \c T = { \c float, \c double }
 */


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
::look_at(const vector_3_<T>& stare_point,
          const vector_3_<T>& up_direction)
{
  // a unit vector in the up direction
  const vector_3_<T> up = normalized(up_direction);
  // a unit vector in the look direction (camera Z-axis)
  const vector_3_<T> z = normalized(stare_point - get_center());

  // the X-axis of the camera is perpendicular to up and z
  vector_3_<T> x = cross_product(-up, z);
  T x_mag = x.magnitude();

  // if the cross product magnitude is small then the up and z vectors are
  // nearly parallel and the up direction is poorly defined.
  if( x_mag < 1e-4)
  {
    std::cerr << "WARNING: camera_::look_at up_direction is "
              << "nearly parallel with the look direction" << std::endl;
  }

  x /= x_mag;
  vector_3_<T> y = normalized(cross_product(z,x));

  T r[] = { x.x(), x.y(), x.z(),
            y.x(), y.y(), y.z(),
            z.x(), z.y(), z.z() };

  matrix_<3,3,T> R(r);
  this->set_rotation(rotation_<T>(R));
}


/// Convert to a 3x4 homogeneous projection matrix
template <typename T>
camera_<T>
::operator matrix_<3,4,T>() const
{
  matrix_<3,4,T> P;
  matrix_<3,3,T> R(this->get_rotation());
  matrix_<3,3,T> K(this->get_intrinsics());
  vector_<3,T>   t(this->get_translation());
  P.update(R);
  P.set_column(3,t);
  return K * P;
}


/// Project a 3D point into a 2D image point
template <typename T>
vector_2_<T>
camera_<T>
::project(const vector_3_<T>& pt) const
{
  return this->intrinsics_.map(this->orientation_ * (pt - this->center_));
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
  matrix_<3,3,T> K, R;
  vector_<3,T> t;
  double d;
  s >> K >> R >> t >> d;
  k.set_intrinsics(camera_intrinsics_<T>(K));
  k.set_rotation(rotation_<T>(R));
  k.set_translation(t);
  return s;
}


#define INSTANTIATE_CAMERA(T) \
template class MAPTK_CORE_EXPORT camera_<T>; \
template MAPTK_CORE_EXPORT std::ostream& operator<<(std::ostream& s, const camera_<T>& c); \
template MAPTK_CORE_EXPORT std::istream& operator>>(std::istream& s, camera_<T>& c)

INSTANTIATE_CAMERA(double);
INSTANTIATE_CAMERA(float);

#undef INSTANTIATE_CAMERA
} // end namespace maptk
