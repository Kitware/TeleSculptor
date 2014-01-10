/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include "detect_features.h"

#include <vector>
#include <maptk/ocv/feature_set.h>
#include <maptk/ocv/image_container.h>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>


namespace maptk
{

namespace ocv
{


/// Private implementation class
class detect_features::priv
{
public:
  /// Constructor
  priv()
  : detector(cv::FeatureDetector::create("GridSURF"))
  {
  }

  priv(const priv& other)
  : detector(cv::FeatureDetector::create("GridSURF"))
  {
  }

  /// the feature detector algorithm
  cv::Ptr<cv::FeatureDetector> detector;
};


/// Constructor
detect_features
::detect_features()
: d_(new priv)
{
}


/// Copy Constructor
detect_features
::detect_features(const detect_features& other)
: d_(new priv(*other.d_))
{
}


/// Destructor
detect_features
::~detect_features()
{
}


/// Extract a set of image features from the provided image
/// \param image_data contains the image data to process
/// \returns a set of image features
feature_set_sptr
detect_features
::detect(image_container_sptr image_data) const
{
  cv::Mat img = maptk::ocv::image_container::maptk_to_ocv(image_data->get_image());
  std::vector<cv::KeyPoint> keypoints;
  d_->detector->detect(img, keypoints);
  return feature_set_sptr(new feature_set(keypoints));
}


} // end namespace ocv

} // end namespace maptk
