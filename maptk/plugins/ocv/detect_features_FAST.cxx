/*ckwg +29
 * Copyright 2016 by Kitware, Inc.
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
 * \brief OCV FAST feature detector wrapper implementation
 */

#include "detect_features_FAST.h"

using namespace kwiver::vital;

namespace kwiver {
namespace maptk {
namespace ocv {


class detect_features_FAST::priv
{
public:
  /// Constructor
  priv()
    : threshold(10),
      nonmaxSuppression(true)
  {
#ifdef MAPTK_HAS_OPENCV_VER_3
    neighborhood_type = cv::FastFeatureDetector::TYPE_9_16;
#endif
  }

  /// Copy Constructor
  priv( const priv &other )
    : threshold( other.threshold ),
      nonmaxSuppression( other.nonmaxSuppression )
  {
#ifdef MAPTK_HAS_OPENCV_VER_3
    // Also update neighborhood type if we're using a 3.x version
    neighborhood_type = other.neighborhood_type;
#endif
  }

  /// Create a new FAST detector instance with the current parameter values
  cv::Ptr<cv::FastFeatureDetector> create_new_detector() const
  {
#ifndef MAPTK_HAS_OPENCV_VER_3
    // 2.4.x version constructor
    return new cv::FastFeatureDetector(threshold, nonmaxSuppression);
#else
    // 3.x version constructor
    return cv::FastFeatureDetector::create(threshold, nonmaxSuppression,
                                           neighborhood_type);
#endif
  }

  /// Update the parameters of teh given detector with the currently set values
  void update_detector_params(cv::Ptr<cv::FastFeatureDetector> detector) const
  {
#ifndef MAPTK_HAS_OPENCV_VER_3
    detector->set("threshold", threshold);
    detector->set("nonmaxSuppression", nonmaxSuppression);
#else
    detector->setThreshold(threshold);
    detector->setNonmaxSuppression(nonmaxSuppression);
    detector->setType(neighborhood_type);
#endif
  }

  int threshold;
  bool nonmaxSuppression;
#ifdef MAPTK_HAS_OPENCV_VER_3
  int neighborhood_type;
#endif
};


/// Constructor
detect_features_FAST
::detect_features_FAST()
  : p_(new priv)
{
  attach_logger("maptk::ocv::detect_features_FAST");
  detector = p_->create_new_detector();
}


/// Copy Constructor
detect_features_FAST
::detect_features_FAST(const detect_features_FAST &other)
  : p_(new priv(*other.p_))
{
  attach_logger("maptk::ocv::detect_features_FAST");
  detector = p_->create_new_detector();
}


/// Destructor
detect_features_FAST
::~detect_features_FAST()
{
}


vital::config_block_sptr
detect_features_FAST
::get_configuration() const
{
  vital::config_block_sptr config = maptk::ocv::detect_features::get_configuration();
  return config;
}


void
detect_features_FAST
::set_configuration(vital::config_block_sptr config)
{
}


bool
detect_features_FAST
::check_configuration(vital::config_block_sptr config) const
{
  return true;
}


} // end namespace ocv
} // end namespace maptk
} // end namespace kwiver
