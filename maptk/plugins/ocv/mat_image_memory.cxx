/*ckwg +29
 * Copyright 2013-2015 by Kitware, Inc.
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
 * \brief OCV mat_image_memory implementation
 */

#include "mat_image_memory.h"


namespace kwiver {
namespace maptk {

namespace ocv
{


/// Constructor - allocates n bytes
/**
 * Base on how the cv::Mat constructor that taked ranges is implemented
 * (sub-matrix construction), data is a pointer with value greater than or equal
 * to datastart. Thus, the start of the global data is datastart and the start
 * of the given matrix's window is data.
 */
mat_image_memory
::mat_image_memory(const cv::Mat& m)
: mat_data_( const_cast<unsigned char*>(m.datastart) ),
#ifndef MAPTK_HAS_OPENCV_VER_3
  mat_refcount_(m.refcount)
#else
  u_( m.u )
#endif
{
  assert(m.depth() == CV_8U);
  assert(!m.allocator);
#ifndef MAPTK_HAS_OPENCV_VER_3
  CV_XADD(this->mat_refcount_, 1);
#else
  if ( u_ )
  {
    CV_XADD(&u_->refcount, 1);
  }
#endif
  this->size_ = static_cast<size_t>( m.rows * m.step );
}


/// Destructor
mat_image_memory
::~mat_image_memory()
{
#ifndef MAPTK_HAS_OPENCV_VER_3
  if( this->mat_refcount_ && CV_XADD(this->mat_refcount_, -1) == 1 )
#else
  if( u_ && CV_XADD( &u_->refcount, -1 ) == 1 )
#endif
  {
    cv::fastFree(this->mat_data_);
  }
}


} // end namespace ocv

} // end namespace maptk
} // end namespace kwiver
