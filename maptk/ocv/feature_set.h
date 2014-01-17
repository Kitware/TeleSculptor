/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_OCV_FEATURE_SET_H_
#define MAPTK_OCV_FEATURE_SET_H_

#include "ocv_config.h"
#include <maptk/core/feature_set.h>

#include <opencv2/features2d/features2d.hpp>

namespace maptk
{

namespace ocv
{


/// A concrete feature set that wraps OpenCV KeyPoints
class MAPTK_OCV_EXPORT feature_set
  : public maptk::feature_set
{
public:
  /// Default Constructor
  feature_set() {}

  /// Constructor from a vector of cv::KeyPoints
  explicit feature_set(const std::vector<cv::KeyPoint>& features)
  : data_(features) {}

  /// Return the number of feature in the set
  virtual size_t size() const { return data_.size(); }

  /// Return a vector of feature shared pointers
  virtual std::vector<feature_sptr> features() const;

  /// Return the underlying OpenCV vector of cv::KeyPoints
  const std::vector<cv::KeyPoint>& ocv_keypoints() const { return data_; }

protected:

  /// The vector of KeyPoints
  std::vector<cv::KeyPoint> data_;
};


/// Convert any feature set to a vector of OpenCV cv::KeyPoints
MAPTK_OCV_EXPORT std::vector<cv::KeyPoint>
features_to_ocv_keypoints(const maptk::feature_set& features);


} // end namespace ocv

} // end namespace maptk


#endif // MAPTK_OCV_FEATURE_SET_H_
