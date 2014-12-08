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
 * \brief OCV mat_image_memory interface
 */

#ifndef MAPTK_MAT_IMAGE_MEMORY_H_
#define MAPTK_MAT_IMAGE_MEMORY_H_

#include <maptk/image.h>

#include <maptk/plugins/ocv/ocv_config.h>

#include <opencv2/core/core.hpp>


namespace maptk
{

namespace ocv
{

/// An image memory class that shares memory with OpenCV using reference counting
class MAPTK_OCV_EXPORT mat_image_memory
  : public image_memory
{
public:
  /// Constructor - allocates n bytes
  mat_image_memory(const cv::Mat& m);

  /// Destructor
  virtual ~mat_image_memory();

  /// Return a pointer to the allocated memory
  virtual void* data() { return this->mat_data_; }

  /// Return the OpenCV reference counter
  int* get_ref_counter() const { return this->mat_refcount_; }

protected:
  /// The cv::Mat data
  unsigned char* mat_data_;

  /// The ref count shared with cv::Mat
  int* mat_refcount_;
};


} // end namespace ocv

} // end namespace maptk


#endif // MAPTK_MAT_IMAGE_MEMORY_H_
