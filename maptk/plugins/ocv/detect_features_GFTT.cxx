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
 * \brief OCV GFTT feature detector wrapper implementation
 */

#include "detect_features_GFTT.h"

using namespace kwiver::vital;

namespace kwiver {
namespace maptk {
namespace ocv {


class detect_features_GFTT::priv
{
public:
  /// Constructor
  priv()
    : max_corners( 1000 ),
      quality_level( 0.01 ),
      min_distance( 1.0 ),
      block_size( 3 ),
      use_harris_detector( false ),
      k( 0.04 )
  {
  }

  /// Copy Constructor
  priv( const priv &other )
    : max_corners( other.max_corners ),
      quality_level( other.quality_level ),
      min_distance( other.min_distance ),
      block_size( other.block_size ),
      use_harris_detector( other.use_harris_detector ),
      k( other.k )
  {
  }

  /// Create a new GFTT detector instance with the current parameter values
  cv::Ptr<cv::GFTTDetector> create() const
  {
#ifndef MAPTK_HAS_OPENCV_VER_3
    return cv::Ptr<cv::GFTTDetector>(
      new cv::GFTTDetector( max_corners, quality_level, min_distance,
                            block_size, use_harris_detector, k )
    );
#else
    return cv::GFTTDetector::create( max_corners, quality_level, min_distance,
                                     block_size, use_harris_detector, k );
#endif
  }

#ifdef MAPTK_HAS_OPENCV_VER_3
  /// Update the parameters of the given detector with the currently set values
  /**
   * Returns false if the algo could not be updating, requiring recreation.
   */
  bool update(cv::Ptr<cv::GFTTDetector> a) const
  {
    a->setMaxFeatures( max_corners );
    a->setQualityLevel( quality_level );
    a->setMinDistance( min_distance );
    a->setBlockSize( block_size );
    a->setHarrisDetector( use_harris_detector );
    a->setK( k );
    return true;
  }
#endif

  /// Update given config block with currently set parameter values
  void update_config( config_block_sptr config ) const
  {
    config->set_value( "max_corners", max_corners );
    config->set_value( "quality_level", quality_level );
    config->set_value( "min_distance", min_distance );
    config->set_value( "block_size", block_size );
    config->set_value( "use_harris_detector", use_harris_detector );
    config->set_value( "k", k );
  }

  /// Set parameter values based on given config block
  void set_config( config_block_sptr const &config )
  {
    max_corners = config->get_value<int>( "max_corners" );
    quality_level = config->get_value<double>( "quality_level" );
    min_distance = config->get_value<double>( "min_distance" );
    block_size = config->get_value<int>( "block_size" );
    use_harris_detector = config->get_value<bool>( "use_harris_detector" );
    k = config->get_value<double>( "k" );
  }

  /// Parameters
  int max_corners;
  double quality_level;
  double min_distance;
  int block_size;
  bool use_harris_detector;
  double k;
};


detect_features_GFTT
::detect_features_GFTT()
  : p_( new priv )
{
  attach_logger( "maptk.ocv.GFTT" );
  detector = p_->create();
}


detect_features_GFTT
::detect_features_GFTT(const detect_features_GFTT &other)
  : p_( new priv( *other.p_ ) )
{
  attach_logger( "maptk.ocv.GFTT" );
  detector = p_->create();
}


detect_features_GFTT
::~detect_features_GFTT()
{
}


vital::config_block_sptr
detect_features_GFTT
::get_configuration() const
{
  config_block_sptr config = ocv::detect_features::get_configuration();
  p_->update_config( config );
  return config;
}


void
detect_features_GFTT
::set_configuration(vital::config_block_sptr config)
{
  config_block_sptr c = get_configuration();
  c->merge_config( config );
  p_->set_config( c );
#ifndef MAPTK_HAS_OPENCV_VER_3
  // since 2.4.x does not have params set for everything that's given to the
  // constructor, lets just remake the algo instance.
  detector = p_->create();
#else
  p_->update( detector.dynamicCast<cv::GFTTDetector>() );
#endif
}


bool
detect_features_GFTT
::check_configuration(vital::config_block_sptr config) const
{
  // Nothing to explicitly check
  return true;
}


} // end namespace ocv
} // end namespace maptk
} // end namespace kwiver
