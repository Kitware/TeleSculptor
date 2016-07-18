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
 * \brief OCV Star feature detector wrapper implementation
 */

#include "detect_features_STAR.h"

#if ! defined(MAPTK_HAS_OPENCV_VER_3) || defined(HAVE_OPENCV_XFEATURES2D)

#ifndef MAPTK_HAS_OPENCV_VER_3
typedef cv::StarDetector cv_STAR_t;
#else
#include <opencv2/xfeatures2d.hpp>
typedef cv::xfeatures2d::StarDetector cv_STAR_t;
#endif

using namespace kwiver::vital;

namespace kwiver {
namespace maptk {
namespace ocv {


class detect_features_STAR::priv
{
public:
  priv()
    : max_size( 45 )
    , response_threshold( 30 )
    , line_threshold_projected( 10 )
    , line_threshold_binarized( 8 )
    , suppress_nonmax_size( 5 )
  {
  }

  priv( priv const &other )
    : max_size( other.max_size )
    , response_threshold( other.response_threshold )
    , line_threshold_projected( other.line_threshold_projected )
    , line_threshold_binarized( other.line_threshold_binarized )
    , suppress_nonmax_size( other.suppress_nonmax_size )
  {
  }

  cv::Ptr<cv_STAR_t> create() const
  {
#ifndef MAPTK_HAS_OPENCV_VER_3
    return cv::Ptr<cv_STAR_t>(
      new cv_STAR_t( max_size, response_threshold, line_threshold_projected,
                     line_threshold_binarized, suppress_nonmax_size )
    );
#else
    return cv_STAR_t::create( max_size, response_threshold,
                              line_threshold_projected,
                              line_threshold_binarized, suppress_nonmax_size );
#endif
  }

#ifndef MAPTK_HAS_OPENCV_VER_3
  void update( cv::Ptr<cv_STAR_t> a ) const
  {
    a->set( "maxSize", max_size );
    a->set( "responseThreshold", response_threshold );
    a->set( "lineThresholdProjected", line_threshold_projected );
    a->set( "lineThresholdBinarized", line_threshold_binarized );
    a->set( "suppressNonmaxSize", suppress_nonmax_size );
  }
#endif

  void update_config( config_block_sptr config ) const
  {
    config->set_value( "max_size", max_size );
    config->set_value( "response_threshold", response_threshold );
    config->set_value( "line_threshold_projected", line_threshold_projected );
    config->set_value( "line_threshold_binarized", line_threshold_binarized );
    config->set_value( "suppress_nonmax_size", suppress_nonmax_size );
  }

  void set_config( config_block_sptr config )
  {
    max_size = config->get_value<int>( "max_size" );
    response_threshold = config->get_value<int>( "response_threshold" );
    line_threshold_projected = config->get_value<int>( "line_threshold_projected" );
    line_threshold_binarized = config->get_value<int>( "line_threshold_binarized" );
    suppress_nonmax_size = config->get_value<int>( "suppress_nonmax_size" );
  }

  // Parameters
  int max_size;
  int response_threshold;
  int line_threshold_projected;
  int line_threshold_binarized;
  int suppress_nonmax_size;
};


detect_features_STAR
::detect_features_STAR()
  : p_( new priv )
{
  attach_logger( "maptk.ocv.star" );
  detector = p_->create();
}


detect_features_STAR
::detect_features_STAR(detect_features_STAR const &other)
  : p_( new priv( *other.p_ ) )
{
  attach_logger( "maptk.ocv.star" );
  detector = p_->create();
}


detect_features_STAR
::~detect_features_STAR()
{
}


vital::config_block_sptr
detect_features_STAR
::get_configuration() const
{
  config_block_sptr config = ocv::detect_features::get_configuration();
  p_->update_config( config );
  return config;
}


void
detect_features_STAR
::set_configuration(vital::config_block_sptr config)
{
  config_block_sptr c = get_configuration();
  c->merge_config( config );
  p_->set_config( c );
#ifndef MAPTK_HAS_OPENCV_VER_3
  p_->update( detector );
#else
  detector = p_->create();
#endif
}


bool
detect_features_STAR
::check_configuration(vital::config_block_sptr config) const
{
  return true;
}


} // end namespace ocv
} // end namespace maptk
} // end namespace kwiver

#endif //has OCV support
