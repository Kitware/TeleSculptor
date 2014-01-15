/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include "feature.h"


namespace maptk
{


/// output stream operator for a feature base class
std::ostream& operator<<(std::ostream& s, const feature& f)
{
  // TODO include covariance once stream operators are defined
  s << f.loc() << " "
    << f.magnitude() << " "
    << f.scale() << " "
    << f.angle();
  return s;
}


/// Default Constructor
template <typename T>
feature_<T>
::feature_()
: loc_(0,0),
  magnitude_(0),
  scale_(1),
  angle_(0)
{
}


/// Constructor for a feature
template <typename T>
feature_<T>
::feature_(const vector_2_<T>& loc, T mag, T scale, T angle)
: loc_(loc),
  magnitude_(mag),
  scale_(scale),
  angle_(angle)
{
}


/// output stream operator for a feature
template <typename T>
std::ostream&  operator<<(std::ostream& s, const feature_<T>& f)
{
  // TODO include covariance once stream operators are defined
  s << f.get_loc() << " "
    << f.get_magnitude() << " "
    << f.get_scale() << " "
    << f.get_angle();
  return s;
}


/// input stream operator for a feature
template <typename T>
std::istream&  operator>>(std::istream& s, feature_<T>& f)
{
  // TODO include covariance once stream operators are defined
  vector_2_<T> loc;
  T magnitude;
  T scale;
  T angle;
  s >> loc
    >> magnitude
    >> scale
    >> angle;
  f.set_loc(loc);
  f.set_magnitude(magnitude);
  f.set_scale(scale);
  f.set_angle(angle);
  return s;
}


#define INSTANTIATE_FEATURE(T) \
template class MAPTK_CORE_EXPORT feature_<T>; \
template MAPTK_CORE_EXPORT std::ostream& operator<<(std::ostream& s, const feature_<T>& f); \
template MAPTK_CORE_EXPORT std::istream& operator>>(std::istream& s, feature_<T>& f)

INSTANTIATE_FEATURE(double);
INSTANTIATE_FEATURE(float);

#undef INSTANTIATE_FEATURE

} // end namespace maptk
