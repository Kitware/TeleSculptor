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
 * \brief OCV BRISK feature detector and extractor wrapper implementation
 */

#include "feature_detect_extract_BRISK.h"

using namespace kwiver::vital;

namespace kwiver {
namespace maptk {
namespace ocv {


namespace {

/// Common BRISK private implementation class
/**
 * TODO: Support for custom pattern configuration and constructor?
 */
class priv
{
public:
  /// Constructor
  priv()
     : threshold(30),
       octaves(3),
       pattern_scale(1.0f)
  {
  }

  /// Copy Constructor
  priv(priv const &other)
     : threshold(other.threshold),
       octaves(other.octaves),
       pattern_scale(other.pattern_scale)
  {
  }

  /// Create new impl instance based on current parameters
  cv::Ptr<cv::BRISK> create() const
  {
#ifndef MAPTK_HAS_OPENCV_VER_3
    return cv::Ptr<cv::BRISK>(
      new cv::BRISK( threshold, octaves, pattern_scale )
    );
#else
    return cv::BRISK::create( threshold, octaves, pattern_scale );
#endif
  }

  /// Update given configuration with current parameter keys and values
  void update_configuration(vital::config_block_sptr &config) const
  {
    config->set_value("threshold", threshold,
                      "AGAST detection threshold score.");
    config->set_value("octaves", octaves,
                      "detection octaves. Use 0 to do single scale.");
    config->set_value("pattern_scale", pattern_scale,
                      "apply this scale to the pattern used for sampling the "
                         "neighbourhood of a keypoint.");
  }

  /// Update parameters based on the given config-block
  void set_configuration(vital::config_block_sptr const &config)
  {
    threshold = config->get_value<int>("threshold");
    octaves = config->get_value<int>("octaves");
    pattern_scale = config->get_value<float>("pattern_scale");
  }

  /// Parameters
  int threshold;
  int octaves;
  float pattern_scale;
};

} // end anon namespace


/// Private implementation class for BRISK feature detection
class detect_features_BRISK::priv
  : public ocv::priv
{
};

/// Private implementation class for BRISK descriptor extraction
class extract_descriptors_BRISK::priv
  : public ocv::priv
{
};


detect_features_BRISK
::detect_features_BRISK()
  : p_( new priv )
{
  attach_logger("maptk.ocv.BRISK");
  detector = p_->create();
}


detect_features_BRISK
::detect_features_BRISK(detect_features_BRISK const &other)
  : p_( new priv( *other.p_ ) )
{
  attach_logger("maptk.ocv.BRISK");
  detector = p_->create();
}


detect_features_BRISK
::~detect_features_BRISK()
{
}


vital::config_block_sptr
detect_features_BRISK
::get_configuration() const
{
  config_block_sptr config = detect_features::get_configuration();
  p_->update_configuration(config);
  return config;
}


void
detect_features_BRISK
::set_configuration(vital::config_block_sptr config)
{
  config_block_sptr c = get_configuration();
  c->merge_config(config);
  p_->set_configuration(c);
  detector = p_->create();
}


bool
detect_features_BRISK
::check_configuration(vital::config_block_sptr config) const
{
  return true;
}


extract_descriptors_BRISK
::extract_descriptors_BRISK()
  : p_( new priv )
{
  attach_logger("maptk.ocv.BRISK");
  extractor = p_->create();
}


extract_descriptors_BRISK
::extract_descriptors_BRISK(extract_descriptors_BRISK const &other)
   : p_( new priv( *other.p_ ) )
{
  attach_logger("maptk.ocv.BRISK");
  extractor = p_->create();
}


extract_descriptors_BRISK
::~extract_descriptors_BRISK()
{
}


vital::config_block_sptr
extract_descriptors_BRISK
::get_configuration() const
{
  config_block_sptr config = extract_descriptors::get_configuration();
  p_->update_configuration(config);
  return config;
}


void
extract_descriptors_BRISK
::set_configuration(vital::config_block_sptr config)
{
  config_block_sptr c = get_configuration();
  c->merge_config(config);
  p_->set_configuration(c);
  extractor = p_->create();
}


bool
extract_descriptors_BRISK
::check_configuration(vital::config_block_sptr config) const
{
  return true;
}


} // end namespace ocv
} // end namespace maptk
} // end namespace kwiver
