/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_OCV_IMAGE_CONTAINER_H_
#define MAPTK_OCV_IMAGE_CONTAINER_H_

#include "ocv_config.h"
#include <maptk/core/image_container.h>

#include <opencv2/core/core.hpp>

namespace maptk
{

namespace ocv
{


/// This image container wraps a cv::Mat
class MAPTK_OCV_EXPORT image_container
  : public maptk::image_container
{
public:

  /// Constructor - from a cv::Mat
  explicit image_container(const cv::Mat& d)
  : data_(d) {}

  /// Constructor - convert maptk image to cv::Mat
  explicit image_container(const image& maptk_image)
  : data_(maptk_to_ocv(maptk_image)) {}

  /// Constructor - convert base image container to cv::Mat
  explicit image_container(const maptk::image_container& image_cont);

  /// Copy Constructor
  image_container(const maptk::ocv::image_container& other)
  : data_(other.data_) {}

  /// The size of the image data in bytes
  /// This size includes all allocated image memory,
  /// which could be larger than width*height*depth.
  virtual size_t size() const;

  /// The width of the image in pixels
  virtual size_t width() const { return data_.rows; }

  /// The height of the image in pixels
  virtual size_t height() const { return data_.cols; }

  /// The depth (or number of channels) of the image
  virtual size_t depth() const { return data_.channels(); }

  /// Get and in-memory image class to access the data
  virtual image get_image() const { return ocv_to_maptk(data_); }

  /// Access the underlying cv::Mat data structure
  cv::Mat get_Mat() const { return data_; }

  /// Convert an OpenCV cv::Mat to a MAPTK image
  static image ocv_to_maptk(const cv::Mat& img);

  /// Convert a MAPTK image to an OpenCV cv::Mat
  static cv::Mat maptk_to_ocv(const image& img);

protected:

  cv::Mat data_;
};


/// Extract a cv::Mat from any image container
/**
 * If \a img is actually an maptk::ocv::image_container then
 * return the underlying cv::Mat.  Otherwise, convert the image data
 * to cv:Mat by shallow copy (if possible) or deep copy as a last resort.
 */
MAPTK_OCV_EXPORT cv::Mat image_container_to_ocv_matrix(const maptk::image_container& img);


} // end namespace ocv

} // end namespace maptk


#endif // MAPTK_OCV_IMAGE_CONTAINER_H_
