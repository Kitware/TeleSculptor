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
 * \brief OCV image_container inteface
 */

#ifndef MAPTK_PLUGINS_OCV_IMAGE_CONTAINER_H_
#define MAPTK_PLUGINS_OCV_IMAGE_CONTAINER_H_


#include <vital/vital_config.h>
#include <maptk/plugins/ocv/maptk_ocv_export.h>

#include <opencv2/core/core.hpp>

#include <vital/types/image_container.h>


namespace kwiver {
namespace maptk {

namespace ocv
{


/// This image container wraps a cv::Mat
class MAPTK_OCV_EXPORT image_container
  : public vital::image_container
{
public:

  /// Constructor - from a cv::Mat
  explicit image_container(const cv::Mat& d)
  : data_(d) {}

  /// Constructor - convert kwiver image to cv::Mat
  explicit image_container(const vital::image& maptk_image)
  : data_(maptk_to_ocv(maptk_image)) {}

  /// Constructor - convert base image container to cv::Mat
  explicit image_container(const vital::image_container& image_cont);

  /// Copy Constructor
  image_container(const maptk::ocv::image_container& other)
  : data_(other.data_) {}

  /// The size of the image data in bytes
  /**
   * This size includes all allocated image memory,
   * which could be larger than width*height*depth.
   */
  virtual size_t size() const;

  /// The width of the image in pixels
  virtual size_t width() const { return data_.cols; }

  /// The height of the image in pixels
  virtual size_t height() const { return data_.rows; }

  /// The depth (or number of channels) of the image
  virtual size_t depth() const { return data_.channels(); }

  /// Get and in-memory image class to access the data
  virtual vital::image get_image() const { return ocv_to_maptk(data_); }

  /// Access the underlying cv::Mat data structure
  cv::Mat get_Mat() const { return data_; }

  /// Convert an OpenCV cv::Mat to a MAPTK image
  static vital::image ocv_to_maptk(const cv::Mat& img);

  /// Convert a MAPTK image to an OpenCV cv::Mat
  static cv::Mat maptk_to_ocv(const vital::image& img);

protected:
  /// image data
  cv::Mat data_;
};


/// Extract a cv::Mat from any image container
/**
 * If \a img is actually an maptk::ocv::image_container then
 * return the underlying cv::Mat.  Otherwise, convert the image data
 * to cv:Mat by shallow copy (if possible) or deep copy as a last resort.
 *
 * \param img Image container to convert to cv::mat
 */
MAPTK_OCV_EXPORT cv::Mat image_container_to_ocv_matrix(const vital::image_container& img);


} // end namespace ocv

} // end namespace maptk
} // end namespace kwiver


#endif // MAPTK_PLUGINS_OCV_IMAGE_CONTAINER_H_
