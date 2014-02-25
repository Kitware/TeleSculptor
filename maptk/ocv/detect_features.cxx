/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief OCV detect_features algorithm implementation
 */

#include "detect_features.h"

#include <vector>
#include <maptk/ocv/feature_set.h>
#include <maptk/ocv/image_container.h>
#include <maptk/ocv/ocv_algo_tools.h>
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
  : detector(cv::FeatureDetector::create("SURF"))
  {
  }

  priv(const priv& other)
  : detector(cv::FeatureDetector::create("SURF"))
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


/// Get this algorithm's \link maptk::config_block configuration block \endlink
config_block_sptr
detect_features
::get_configuration() const
{
  // base configuration block
  config_block_sptr config = algorithm::get_configuration();

  get_nested_ocv_algo_configuration("detector", config, d_->detector);

  return config;
}


/// Set this algorithm's properties via a config block
void
detect_features
::set_configuration(config_block_sptr config)
{
  set_nested_ocv_algo_configuration(
      "detector", config, d_->detector);
}


/// Check that the algorithm's configuration config_block is valid
bool
detect_features
::check_configuration(config_block_sptr config) const
{
  bool nested_ok =
    check_nested_ocv_algo_configuration<cv::FeatureDetector>(
        "detector", config);

  return nested_ok;
}


/// Extract a set of image features from the provided image
feature_set_sptr
detect_features
::detect(image_container_sptr image_data) const
{
  cv::Mat img = ocv::image_container::maptk_to_ocv(image_data->get_image());
  std::vector<cv::KeyPoint> keypoints;
  d_->detector->detect(img, keypoints);
  return feature_set_sptr(new feature_set(keypoints));
}


} // end namespace ocv

} // end namespace maptk
