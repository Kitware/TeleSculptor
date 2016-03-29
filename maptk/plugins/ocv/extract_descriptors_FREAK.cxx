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
 * \brief OCV FREAK descriptor extractor wrapper implementation
 */

#include "extract_descriptors_FREAK.h"

#if ! defined(MAPTK_HAS_OPENCV_VER_3) || defined(HAVE_OPENCV_XFEATURES2D)

// typedef FREAK into a common symbol
#ifndef MAPTK_HAS_OPENCV_VER_3
typedef cv::FREAK cv_FREAK_t;
#else
#include <opencv2/xfeatures2d.hpp>
typedef cv::xfeatures2d::FREAK cv_FREAK_t;
#endif

namespace kwiver {
namespace maptk {
namespace ocv {


class extract_descriptors_FREAK::priv
{
public:
  /// Constructor
  priv()
    : orientation_normalized( true ),
      scale_normalized( true ),
      pattern_scale( 22.0f ),
      n_octaves( 4 )
  {
  }

  /// Copy constructor
  priv( priv const &other )
      : orientation_normalized( other.orientation_normalized ),
        scale_normalized( other.scale_normalized ),
        pattern_scale( other.pattern_scale ),
        n_octaves( other.n_octaves )
  {
  }

  /// Create new cv::Ptr algo instance
  cv::Ptr<cv_FREAK_t> create() const
  {
#ifndef MAPTK_HAS_OPENCV_VER_3
    return cv::Ptr<cv_FREAK_t>(
        new cv_FREAK_t( orientation_normalized, scale_normalized, pattern_scale,
                        n_octaves )
    );
#else
    return cv_FREAK_t::create( orientation_normalized, scale_normalized,
                               pattern_scale, n_octaves );
#endif
  }

#ifndef MAPTK_HAS_OPENCV_VER_3
  /// Update algorithm instance with current parameter values
  void update_algo( cv::Ptr<cv_FREAK_t> freak ) const
  {
    freak->set( "orientationNormalized", orientation_normalized );
    freak->set( "scaleNormalized", scale_normalized );
    freak->set( "patternScale", pattern_scale );
    freak->set( "nbOctave", n_octaves );
  }
#endif

  /// Set current parameter values to the given config block
  void update_config( vital::config_block_sptr &config ) const
  {
    config->set_value( "orientation_normalized", orientation_normalized,
                       "enable orientation normalization" );
    config->set_value( "scale_normalized", scale_normalized,
                       "enable scale normalization" );
    config->set_value( "pattern_scale", pattern_scale,
                       "scaling of the description pattern" );
    config->set_value( "n_octaves", n_octaves,
                       "number of octaves covered by the detected keypoints" );
  }

  /// Set our parameters based on the given config block
  void set_config( vital::config_block_sptr const &config )
  {
    orientation_normalized = config->get_value<bool>("orientation_normalized");
    scale_normalized = config->get_value<bool>("scale_normalized");
    pattern_scale = config->get_value<float>("pattern_scale");
    n_octaves = config->get_value<int>("n_octaves");
  }

  /// Params
  bool orientation_normalized;
  bool scale_normalized;
  float pattern_scale;
  int n_octaves;
};


/// Constructor
extract_descriptors_FREAK
::extract_descriptors_FREAK()
    : p_( new priv )
{
  attach_logger("maptk.ocv.FREAK");
  extractor = p_->create();
}


/// Copy Constructor
extract_descriptors_FREAK
::extract_descriptors_FREAK(extract_descriptors_FREAK const &other)
: p_( new priv(*other.p_) )
{
  attach_logger("maptk.ocv.FREAK");
  extractor = p_->create();
}


/// Destructor
extract_descriptors_FREAK
::~extract_descriptors_FREAK()
{
}


vital::config_block_sptr
extract_descriptors_FREAK
::get_configuration() const
{
  vital::config_block_sptr config = extract_descriptors::get_configuration();
  p_->update_config(config);
  return config;
}


void
extract_descriptors_FREAK
::set_configuration(vital::config_block_sptr config)
{
  vital::config_block_sptr c = get_configuration();
  c->merge_config( config );
  p_->set_config( c );
#ifndef MAPTK_HAS_OPENCV_VER_3
  p_->update_algo( extractor );
#else
  extractor = p_->create();
#endif
}


bool
extract_descriptors_FREAK
::check_configuration(vital::config_block_sptr in_config) const
{
  return true;
}


} // end namespace ocv
} // end namespace maptk
} // end namespace kwiver

#endif //! defined(MAPTK_HAS_OPENCV_VER_3) || defined(HAVE_OPENCV_XFEATURES2D)
