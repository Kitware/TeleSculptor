/*ckwg +5
 * Copyright 2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include "camera.h"

namespace maptk
{


template <typename T>
std::ostream&  operator<<(std::ostream& s, const camera_<T>& k)
{
  // TODO: implement me
  return s;
}


/// input stream operator for a camera intrinsics
template <typename T>
std::istream&  operator>>(std::istream& s, camera_<T>& k)
{
  // TODO: implement me
  return s;
}


#define INSTANTIATE_CAMERA(T) \
template class camera_<T>; \
template std::ostream&  operator<<(std::ostream& s, const camera_<T>& c); \
template std::istream&  operator>>(std::istream& s, camera_<T>& c)

INSTANTIATE_CAMERA(double);
INSTANTIATE_CAMERA(float);

#undef INSTANTIATE_CAMERA
} // end namespace maptk
