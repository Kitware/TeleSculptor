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


/// output stream operator for a matrix
template <typename T, int M, int N>
std::ostream&  operator<<(std::ostream& s, const Eigen::Matrix<T,M,N>& m)
{
  for (int i=0; i<M; ++i)
  {
    s << m(i,0);
    for (int j=1; j<N; ++j)
    {
      s << ' ' << m(i,j);
    }
    s << '\n';
  }
  return s;
}

/// input stream operator for a matrix
template <typename T, int M, int N>
std::istream&  operator>>(std::istream& s, Eigen::Matrix<T,M,N>& m)
{
  for (int i=0; i<M; ++i)
  {
    for (int j=0; j<N; ++j)
    {
      s >> std::skipws >> m(i,j);
    }
  }
  return s;
}


/// \cond DoxygenSuppress
#define INSTANTIATE_MATRIX(M,N,T) \
template MAPTK_LIB_EXPORT std::ostream&  operator<<(std::ostream& s, const Eigen::Matrix<T,M,N>& m); \
template MAPTK_LIB_EXPORT std::istream&  operator>>(std::istream& s, Eigen::Matrix<T,M,N>& m)

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
