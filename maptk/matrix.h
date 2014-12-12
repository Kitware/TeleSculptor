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
 * \brief Typedefs for Eigen fixed sized matrices
 */

#ifndef MAPTK_MATRIX_H_
#define MAPTK_MATRIX_H_

#include <maptk/config.h>

#include <iostream>
#include <cstring>

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


/// input stream operator for a matrix
/**
 * \param s an input stream
 * \param m a matrix to stream into
 */
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


} // end namespace maptk


#endif // MAPTK_MATRIX_H_
