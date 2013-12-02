/*ckwg +5
 * Copyright 2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_OCV_DESCRIPTOR_SET_H_
#define MAPTK_OCV_DESCRIPTOR_SET_H_


#include <maptk/core/descriptor_set.h>
#include <opencv2/features2d/features2d.hpp>


namespace maptk
{

namespace ocv
{

/// A concrete descriptor set that wraps OpenCV descriptors.
class descriptor_set
: public maptk::descriptor_set
{
public:
  /// Default Constructor
  descriptor_set() {}

  /// Constructor from an OpenCV descriptor matrix
  explicit descriptor_set(const cv::Mat& descriptor_matrix)
  : data_(descriptor_matrix) {}

  /// Return the number of descriptor in the set
  virtual size_t size() const { return data_.rows; }

  /// Return a vector of descriptor shared pointers
  virtual std::vector<descriptor_sptr> descriptors() const;

  /// Return the native OpenCV descriptors as a matrix
  const cv::Mat& ocv_desc_matrix() const { return data_; }

protected:

  /// The OpenCV matrix of featrues
  cv::Mat data_;
};


/// Convert any descriptor set to an OpenCV cv::Mat
cv::Mat
descriptors_to_ocv_matrix(const maptk::descriptor_set& desc_set);


} // end namespace ocv

} // end namespace maptk


#endif // MAPTK_OCV_DESCRIPTOR_SET_H_
