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
 * \brief Header for \link maptk::matrix_ matrix_<M,N,T> \endlink class
 */

#ifndef MAPTK_MATRIX_H_
#define MAPTK_MATRIX_H_

#include <maptk/config.h>

#include <iostream>
#include <cstring>
#include <cassert>

#include "vector.h"
#include "exceptions.h"

#include <Eigen/Core>


namespace maptk
{


/// \cond DoxygenSuppress
typedef Eigen::Matrix<double, 2, 2> matrix_2x2d;
typedef Eigen::Matrix<float, 2, 2>  matrix_2x2f;
typedef Eigen::Matrix<double, 2, 3> matrix_2x3d;
typedef Eigen::Matrix<float, 2, 3>  matrix_2x3f;
typedef Eigen::Matrix<double, 3, 2> matrix_3x2d;
typedef Eigen::Matrix<float, 3, 2>  matrix_3x2f;
typedef Eigen::Matrix<double, 3, 3> matrix_3x3d;
typedef Eigen::Matrix<float, 3,3>   matrix_3x3f;
typedef Eigen::Matrix<double, 3, 4> matrix_3x4d;
typedef Eigen::Matrix<float, 3, 4>  matrix_3x4f;
typedef Eigen::Matrix<double, 4, 3> matrix_4x3d;
typedef Eigen::Matrix<float, 4, 3>  matrix_4x3f;
typedef Eigen::Matrix<double, 4, 4> matrix_4x4d;
typedef Eigen::Matrix<float, 4, 4>  matrix_4x4f;
/// \endcond

#if 0
/// Compute the determinant of a 2x2 square matrix
template <typename T>
inline
T determinant(const Eigen::Matrix<T,2,2>& m)
{
  const T* d = m.data();
  return m[0]*d[3] - d[1]*d[2];
}

/// Compute the determinant of a 3x3 square matrix
template <typename T>
inline
T determinant(const matrix_<3,3,T>& m)
{
  const T* d = m.data();
  return d[0]*(d[4]*d[8] - d[5]*d[7])
       + d[1]*(d[5]*d[6] - d[3]*d[8])
       + d[2]*(d[3]*d[7] - d[4]*d[6]);
}

/// Compute the inverse of a 2x2 square matrix
template <typename T>
matrix_<2,2,T> inverse(const matrix_<2,2,T>& m)
{
  T det = determinant(m);
  if (det==0)
  {
    throw non_invertible_matrix();
  }
  det = T(1)/det;
  T d[4];
  d[0] = m(1,1)*det;
  d[1] = -m(0,1)*det;
  d[2] = -m(1,0)*det;
  d[3] = m(0,0)*det;
  return matrix_<2,2,T>(d);
}

/// Compute the inverse of a 3x3 square matrix
template <typename T>
matrix_<3,3,T> inverse(const matrix_<3,3,T>& m)
{
  T det = determinant(m);
  if (det==0)
  {
    throw non_invertible_matrix();
  }
  det = T(1)/det;
  T d[9];
  d[0] = (m(1,1)*m(2,2)-m(1,2)*m(2,1))*det;
  d[1] = (m(2,1)*m(0,2)-m(2,2)*m(0,1))*det;
  d[2] = (m(0,1)*m(1,2)-m(0,2)*m(1,1))*det;
  d[3] = (m(1,2)*m(2,0)-m(1,0)*m(2,2))*det;
  d[4] = (m(0,0)*m(2,2)-m(0,2)*m(2,0))*det;
  d[5] = (m(1,0)*m(0,2)-m(1,2)*m(0,0))*det;
  d[6] = (m(1,0)*m(2,1)-m(1,1)*m(2,0))*det;
  d[7] = (m(0,1)*m(2,0)-m(0,0)*m(2,1))*det;
  d[8] = (m(0,0)*m(1,1)-m(0,1)*m(1,0))*det;
  return matrix_<3,3,T>(d);
}

/// Compute the cross_product 3x3 matrix from a 3D vector
/**
 * Produces a matrix such that
 * \code
 *   cross_product(v1) * v2 == cross_product(v1, v2)
 * \endcode
 */
template <typename T>
matrix_<3,3,T> cross_product(const vector_<3,T>& v)
{
  matrix_<3,3,T> x;
  x(0,0) = x(1,1) = x(2,2) = T(0);
  x(0,1) = -v[2];
  x(1,0) =  v[2];
  x(2,0) = -v[1];
  x(0,2) =  v[1];
  x(1,2) = -v[0];
  x(2,1) =  v[0];
  return x;
}

#endif
/// output stream operator for a matrix
/**
 * \param s an output stream
 * \param m a matrix to stream
 */
template <typename T, int M, int N>
MAPTK_LIB_EXPORT std::ostream&  operator<<(std::ostream& s, const Eigen::Matrix<T,M,N>& m);

/// input stream operator for a matrix
/**
 * \param s an input stream
 * \param m a matrix to stream into
 */
template <typename T, int M, int N>
MAPTK_LIB_EXPORT std::istream&  operator>>(std::istream& s, Eigen::Matrix<T,M,N>& m);


} // end namespace maptk


#endif // MAPTK_MATRIX_H_
