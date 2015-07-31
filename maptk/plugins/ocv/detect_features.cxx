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
 * \brief OCV detect_features algorithm implementation
 */

#include "detect_features.h"

#include <iostream>
#include <vector>

#include <vital/exceptions/image.h>
#include <maptk/plugins/ocv/feature_set.h>
#include <maptk/plugins/ocv/image_container.h>
#include <maptk/plugins/ocv/ocv_algo_tools.h>

#include <opencv2/features2d/features2d.hpp>

using namespace kwiver::vital;

namespace kwiver {
namespace maptk {

namespace ocv
{


/// Private implementation class
class detect_features::priv
{
public:
  /// Constructor
  priv()
  : detector(default_detector())
  {
  }

  priv(const priv& other)
  : detector(default_detector())
  {
  }

  /// create the default feature detector
  static cv::Ptr<cv::FeatureDetector> default_detector()
  {
    cv::Ptr<cv::FeatureDetector> det;
    // try the SURF detector first
    det = cv::FeatureDetector::create("SURF");
    if( !det )
    {
      // if SURF is not available (nonfree not built) use ORB
      det = cv::FeatureDetector::create("ORB");
    }
    return det;
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


/// Get this algorithm's \link kwiver::vital::config_block configuration block \endlink
kwiver::vital::config_block_sptr
detect_features
::get_configuration() const
{
  // base configuration block
  kwiver::vital::config_block_sptr config = algorithm::get_configuration();

  get_nested_ocv_algo_configuration("detector", config, d_->detector);

  return config;
}


/// Set this algorithm's properties via a config block
void
detect_features
::set_configuration(kwiver::vital::config_block_sptr config)
{
  set_nested_ocv_algo_configuration(
      "detector", config, d_->detector);
}


/// Check that the algorithm's configuration kwiver::vital::config_block is valid
bool
detect_features
::check_configuration(kwiver::vital::config_block_sptr config) const
{
  bool nested_ok =
    check_nested_ocv_algo_configuration<cv::FeatureDetector>(
        "detector", config);

  return nested_ok;
}


/// Extract a set of image features from the provided image
kwiver::vital::feature_set_sptr
detect_features
::detect(kwiver::vital::image_container_sptr image_data, kwiver::vital::image_container_sptr mask) const
{
  cv::Mat cv_img = ocv::image_container::maptk_to_ocv(image_data->get_image());
  cv::Mat cv_mask;
  std::vector<cv::KeyPoint> keypoints;

  // Only initialize a mask image if the given mask image container contained
  // valid data.
  if( mask && mask->size() > 0 )
  {
    if ( image_data->width() != mask->width() ||
         image_data->height() != mask->height() )
    {
      throw image_size_mismatch_exception(
          "OCV detect feature algorithm given a non-zero mask with mismatched "
          "shape compared to imput image",
          image_data->width(), image_data->height(),
          mask->width(), mask->height()
          );
    }

    // Make sure we make a one-channel cv::Mat
    kwiver::vital::image s = mask->get_image();
    // hijacking memory of given mask image, but only telling the new image
    // object to consider the first channel. See kwiver::vital::image documentation.
    kwiver::vital::image i(s.memory(),
                     static_cast< kwiver::vital::image::byte* >(s.memory()->data()),
                     s.width(),  s.height(), 1 /*depth*/,
                     s.w_step(), s.h_step(), s.d_step());
    cv_mask = ocv::image_container::maptk_to_ocv(i);
  }

  d_->detector->detect(cv_img, keypoints, cv_mask);
  return feature_set_sptr(new feature_set(keypoints));
}


} // end namespace ocv

} // end namespace maptk
} // end namespace kwiver
