/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include "camera.h"
#include <iomanip>

namespace maptk
{


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
template MAPTK_CORE_EXPORT std::ostream& operator<<(std::ostream& s, const camera_<T>& c); \
template MAPTK_CORE_EXPORT std::istream& operator>>(std::istream& s, camera_<T>& c)

INSTANTIATE_CAMERA(double);
INSTANTIATE_CAMERA(float);

#undef INSTANTIATE_CAMERA
} // end namespace maptk
