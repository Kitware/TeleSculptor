/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief OCV mat_image_memory interface
 */

#ifndef MAPTK_MAT_IMAGE_MEMORY_H_
#define MAPTK_MAT_IMAGE_MEMORY_H_

#include "ocv_config.h"
#include <maptk/core/image.h>

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
