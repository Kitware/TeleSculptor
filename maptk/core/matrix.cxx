/*ckwg +5
 * Copyright 2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include "matrix.h"


namespace maptk
{


/// output stream operator for a matrix
template <unsigned M, unsigned N, typename T>
std::ostream&  operator<<(std::ostream& s, const matrix_<M,N,T>& m)
{
  for (unsigned int i=0; i<M; ++i)
  {
    s << m[i][0];
    for (unsigned int j=1; j<N; ++j)
    {
      s << ' ' << m[i][j];
    }
    s << '\n';
  }
  return s;
}

/// input stream operator for a matrix
template <unsigned M, unsigned N, typename T>
std::istream&  operator>>(std::istream& s, matrix_<M,N,T>& m)
{
  for (unsigned int i=0; i<M; ++i)
  {
    for (unsigned int j=0; j<N; ++j)
    {
      s >> m[i][j];
    }
  }
  return s;
}


#define INSTANTIATE_MATRIX(M,N,T) \
template class matrix_<M,N,T>; \
template std::ostream&  operator<<(std::ostream& s, const matrix_<M,N,T>& m); \
template std::istream&  operator>>(std::istream& s, matrix_<M,N,T>& m)

INSTANTIATE_MATRIX(2, 2, double);
INSTANTIATE_MATRIX(2, 2, float);
INSTANTIATE_MATRIX(3, 3, double);
INSTANTIATE_MATRIX(3, 3, float);
INSTANTIATE_MATRIX(3, 4, double);
INSTANTIATE_MATRIX(3, 4, float);
INSTANTIATE_MATRIX(4, 4, double);
INSTANTIATE_MATRIX(4, 4, float);


#undef INSTANTIATE_MATRIX
} // end namespace maptk
