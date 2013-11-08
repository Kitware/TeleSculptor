/*ckwg +5
 * Copyright 2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include "mat_image_memory.h"


namespace maptk
{

namespace ocv
{


/// Constructor - allocates n bytes
mat_image_memory
::mat_image_memory(const cv::Mat& m)
: mat_data_(m.datastart),
  mat_refcount_(m.refcount)
{
  assert(m.depth() == CV_8U);
  assert(!m.allocator);
  CV_XADD(this->mat_refcount_, 1);
  size_ = sizeof(m.data);
}


/// Destructor
mat_image_memory
::~mat_image_memory()
{
  if( this->mat_refcount_ && CV_XADD(this->mat_refcount_, -1) == 1 )
  {
    cv::fastFree(this->mat_data_);
  }
}


} // end namespace ocv

} // end namespace maptk
