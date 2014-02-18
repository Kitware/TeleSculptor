/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include "matrix.h"


/**
 * \file
 * \brief Implementation of \link maptk::matrix_ matrix_<M,N,T> \endlink
 *        for \c T = { \c float, \c double }, \c M = { 2, 3, 4}, and
 *        \c N = { 2, 3, 4}
 */


namespace maptk
{


/// Fill the matrix with this value
template <unsigned M, unsigned N, typename T>
matrix_<M,N,T>&
matrix_<M,N,T>
::fill(const T& value)
{
  T* p = data_[0];
  unsigned int n = M*N;
  while(n--)
  {
    *p++ = value;
  }
  return *this;
}


/// Fill the diagonal with this value
template <unsigned M, unsigned N, typename T>
matrix_<M,N,T>&
matrix_<M,N,T>
::fill_diagonal(const T& value)
{
  for (unsigned int i=0; i < min_dim; ++i)
  {
    this->data_[i][i] = value;
  }
  return *this;
}


/// Set the diagonal to this vector
template <unsigned M, unsigned N, typename T>
matrix_<M,N,T>&
matrix_<M,N,T>
::set_diagonal(const vector_<min_dim,T>& diag)
{
  for (unsigned int i=0; i < min_dim; ++i)
  {
    this->data_[i][i] = diag[i];
  }
  return *this;
}


/// Set the matrix to the identity matrix
/// Extra rows or columns of a non-square matrix are set to zero
template <unsigned M, unsigned N, typename T>
matrix_<M,N,T>&
matrix_<M,N,T>
::set_identity()
{
  this->fill(T(0));
  this->fill_diagonal(T(1));
  return *this;
}


/// Return the transpose of this matrix
template <unsigned M, unsigned N, typename T>
matrix_<N,M,T>
matrix_<M,N,T>
::transpose() const
{
  matrix_<N,M,T> result;
  for (unsigned int i=0; i<M; ++i)
  {
    for (unsigned int j=0; j<N; ++j)
    {
      result(j,i) = this->data_[i][j];
    }
  }
  return result;
}


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
      s >> std::skipws >> m[i][j];
    }
  }
  return s;
}


/// \cond DoxygenSuppress
#define INSTANTIATE_MATRIX(M,N,T) \
template class MAPTK_CORE_EXPORT matrix_<M,N,T>; \
template MAPTK_CORE_EXPORT std::ostream&  operator<<(std::ostream& s, const matrix_<M,N,T>& m); \
template MAPTK_CORE_EXPORT std::istream&  operator>>(std::istream& s, matrix_<M,N,T>& m)

INSTANTIATE_MATRIX(2, 2, double);
INSTANTIATE_MATRIX(2, 2, float);
INSTANTIATE_MATRIX(2, 3, double);
INSTANTIATE_MATRIX(2, 3, float);
INSTANTIATE_MATRIX(3, 2, double);
INSTANTIATE_MATRIX(3, 2, float);
INSTANTIATE_MATRIX(3, 3, double);
INSTANTIATE_MATRIX(3, 3, float);
INSTANTIATE_MATRIX(3, 4, double);
INSTANTIATE_MATRIX(3, 4, float);
INSTANTIATE_MATRIX(4, 3, double);
INSTANTIATE_MATRIX(4, 3, float);
INSTANTIATE_MATRIX(4, 4, double);
INSTANTIATE_MATRIX(4, 4, float);


#undef INSTANTIATE_MATRIX
/// \endcond

} // end namespace maptk
