/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_OCV_MATRIX_H_
#define MAPTK_OCV_MATRIX_H_

#include <maptk/core/matrix.h>

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
