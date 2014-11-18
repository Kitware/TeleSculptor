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
 * \brief OCV templated mat generation function interface/implementation
 */

#ifndef MAPTK_OCV_MATRIX_H_
#define MAPTK_OCV_MATRIX_H_

#include <maptk/matrix.h>

#include <opencv2/opencv.hpp>


namespace maptk
{

namespace ocv
{

/// Convert from an OpenCV cv::Mat to a MAPTK matrix
template <unsigned M, unsigned N, typename T>
inline matrix_<M,N,T>
matrix_from_ocv(const cv::Mat& cvm)
{
  matrix_<M,N,T> m;
  assert(cvm.rows == M);
  assert(cvm.cols == N);
  for (unsigned c=0; c<N; ++c)
  {
    for (unsigned r=0; r<M; ++r)
    {
      m(r,c) = cvm.at<T>(r,c);
    }
  }
  return m;
}


} // end namespace ocv

} // end namespace maptk


#endif // MAPTK_OCV_MATRIX_H_
