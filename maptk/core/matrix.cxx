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
 * \brief Implementation of \link maptk::matrix_ matrix_<M,N,T> \endlink
 *        for \c T = { \c float, \c double }, \c M = { 2, 3, 4}, and
 *        \c N = { 2, 3, 4}
 */

#include "matrix.h"


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
/**
 * Extra rows or columns of a non-square matrix are set to zero
 */
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
