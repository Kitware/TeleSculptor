/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include "vector.h"


/**
 * \file
 * \brief Implementation of \link maptk::vector_ vector_<N,T> \endlink and
 *        related classes for \c T = { \c float, \c double } and
 *        \c N = { 2, 3, 4}
 */


namespace maptk
{


/// output stream operator for a vector
template <unsigned N, typename T>
std::ostream&  operator<<(std::ostream& s, const vector_<N,T>& v)
{
  s << v[0];
  for( unsigned i=1; i<N; ++i )
  {
    s << " " << v[i];
  }
  return s;
}

/// input stream operator for a vector
template <unsigned N, typename T>
std::istream&  operator>>(std::istream& s, vector_<N,T>& v)
{
  for( unsigned i=0; i<N; ++i)
  {
    s >> std::skipws >> v[i];
  }
  return s;
}


#define INSTANTIATE_VECTOR(N,T) \
template class MAPTK_CORE_EXPORT vector_<N,T>; \
template MAPTK_CORE_EXPORT std::ostream&  operator<<(std::ostream& s, const vector_<N,T>& v); \
template MAPTK_CORE_EXPORT std::istream&  operator>>(std::istream& s, vector_<N,T>& v)

INSTANTIATE_VECTOR(2, double);
INSTANTIATE_VECTOR(2, float);
INSTANTIATE_VECTOR(3, double);
INSTANTIATE_VECTOR(3, float);
INSTANTIATE_VECTOR(4, double);
INSTANTIATE_VECTOR(4, float);

#undef INSTANTIATE_VECTOR
} // end namespace maptk
