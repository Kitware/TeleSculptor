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
 * \brief OCV SURF feature detector and extractor wrapper implementation
 */

#include "feature_detect_extract_SURF.h"

#if defined(HAVE_OPENCV_NONFREE) || defined(HAVE_OPENCV_XFEATURES2D)

// Include the correct file and unify different namespace locations of SURF type
// across versions
#ifndef MAPTK_HAS_OPENCV_VER_3
// 2.4.x header location
#include <opencv2/nonfree/features2d.hpp>
typedef cv::SURF cv_SURF_t;
#else
// 3.x header location
#include <opencv2/xfeatures2d/nonfree.hpp>
typedef cv::xfeatures2d::SURF cv_SURF_t;
#endif


using namespace kwiver::vital;

namespace kwiver {
namespace maptk {
namespace ocv {


namespace
{

/// Common ORB private implementation class
class priv
{
public:
  // Cosntructor
  priv()
    : hessian_threshold( 100 )
    , n_octaves( 4 )
    , n_octave_layers( 3 )
    , extended( false )
    , upright( false )
  {
  }

  // Copy Constructor
  priv( priv const &other )
    : hessian_threshold( other.hessian_threshold )
    , n_octaves( other.n_octaves )
    , n_octave_layers( other.n_octave_layers )
    , extended( other.extended )
    , upright( other.upright )
  {
  }

  // Create new algorithm instance from current parameters
  cv::Ptr<cv_SURF_t> create() const
  {
#ifndef MAPTK_HAS_OPENCV_VER_3
    return cv::Ptr<cv_SURF_t>(
      new cv_SURF_t( hessian_threshold, n_octaves, n_octave_layers,
                     extended, upright  )
    );
#else
    return cv_SURF_t::create( hessian_threshold, n_octaves, n_octave_layers,
                              extended, upright );
#endif
  }

  // Update algorithm with current parameter
  void update( cv::Ptr<cv_SURF_t> a ) const
  {
#ifndef MAPTK_HAS_OPENCV_VER_3
    a->set( "hessianThreshold", hessian_threshold );
    a->set( "nOctaves", n_octaves );
    a->set( "nOctaveLayers", n_octave_layers );
    a->set( "extended", extended );
    a->set( "upright", upright );
#else
    a->setHessianThreshold( hessian_threshold );
    a->setNOctaves( n_octaves );
    a->setNOctaveLayers( n_octave_layers );
    a->setExtended( extended );
    a->setUpright( upright );
#endif
  }

  // Update config block with current parameter values
  void update_config( config_block_sptr config ) const
  {
    config->set_value( "hessian_threshold", hessian_threshold,
                       "Threshold for hessian keypoint detector used in SURF" );
    config->set_value( "n_octaves", n_octaves,
                       "Number of pyramid octaves the keypoint detector will "
                       "use." );
    config->set_value( "n_octave_layers", n_octave_layers,
                       "Number of octave layers within each octave." );
    config->set_value( "extended", extended,
                       "Extended descriptor flag (true - use extended "
                       "128-element descriptors; false - use 64-element "
                       "descriptors)." );
    config->set_value( "upright", upright,
                       "Up-right or rotated features flag (true - do not "
                       "compute orientation of features; false - "
                       "compute orientation)." );
  }

  // Set current values based on config block
  void set_config( config_block_sptr config )
  {
    hessian_threshold = config->get_value<int>( "hessian_threshold" );
    n_octaves = config->get_value<int>( "n_octaves");
    n_octave_layers = config->get_value<int>( "n_octave_layers");
    extended = config->get_value<bool>( "extended" );
    upright = config->get_value<bool>( "upright" );
  }

  // Parameters
  double hessian_threshold;
  int n_octaves;
  int n_octave_layers;
  bool extended;
  bool upright;
};

} // end namespace anonymous


class detect_features_SURF::priv
  : public ocv::priv
{
};


class extract_descriptors_SURF::priv
  : public ocv::priv
{
};


detect_features_SURF
::detect_features_SURF()
  : p_( new priv )
{
  attach_logger("maptk.ocv.SURF");
  detector = p_->create();
}


detect_features_SURF
::detect_features_SURF(detect_features_SURF const &other)
  : p_( new priv( *other.p_ ) )
{
  attach_logger("maptk.ocv.SURF");
  detector = p_->create();
}


detect_features_SURF
::~detect_features_SURF()
{
}


vital::config_block_sptr
detect_features_SURF
::get_configuration() const
{
  config_block_sptr config = detect_features::get_configuration();
  p_->update_config( config );
  return config;
}


void
detect_features_SURF
::set_configuration(vital::config_block_sptr config)
{
  config_block_sptr c = get_configuration();
  c->merge_config( config );
  p_->set_config( c );

#ifndef MAPTK_HAS_OPENCV_VER_3
  p_->update( detector );
#else
  p_->update( detector.dynamicCast<cv_SURF_t>() );
#endif
}


bool
detect_features_SURF
::check_configuration(vital::config_block_sptr config) const
{
  return true;
}


extract_descriptors_SURF
::extract_descriptors_SURF()
  : p_( new priv )
{
  attach_logger("maptk.ocv.SURF");
  extractor = p_->create();
}


extract_descriptors_SURF
::extract_descriptors_SURF(extract_descriptors_SURF const &other)
  : p_( new priv( *other.p_ ) )
{
  attach_logger("maptk.ocv.SURF");
  extractor = p_->create();
}


extract_descriptors_SURF
::~extract_descriptors_SURF()
{
}


vital::config_block_sptr
extract_descriptors_SURF
::get_configuration() const
{
  config_block_sptr config = extract_descriptors::get_configuration();
  p_->update_config( config );
  return config;
}


void
extract_descriptors_SURF
::set_configuration(vital::config_block_sptr config)
{
  config_block_sptr c = get_configuration();
  c->merge_config( config );
  p_->set_config( c );

#ifndef MAPTK_HAS_OPENCV_VER_3
  p_->update( extractor );
#else
  p_->update( extractor.dynamicCast<cv_SURF_t>() );
#endif
}


bool
extract_descriptors_SURF
::check_configuration(vital::config_block_sptr config) const
{
  return true;
}


} // end namespace ocv
} // end namespace maptk
} // end namespace kwiver

#endif //defined(HAVE_OPENCV_NONFREE) || defined(HAVE_OPENCV_XFEATURES2D)
