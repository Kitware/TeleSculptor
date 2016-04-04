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
 * \brief OCV SIFT feature detector and extractor wrapper implementation
 */

#include "feature_detect_extract_SIFT.h"

#if defined(HAVE_OPENCV_NONFREE) || defined(HAVE_OPENCV_XFEATURES2D)

// Include the correct file and unify different namespace locations of SIFT type
// across versions
#ifndef MAPTK_HAS_OPENCV_VER_3
// 2.4.x header location
#include <opencv2/nonfree/features2d.hpp>
typedef cv::SIFT cv_SIFT_t;
#else
// 3.x header location
#include <opencv2/xfeatures2d/nonfree.hpp>
typedef cv::xfeatures2d::SIFT cv_SIFT_t;
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
    : n_features( 0 )
    , n_octave_layers( 3 )
    , contrast_threshold( 0.04 )
    , edge_threshold( 10 )
    , sigma( 1.6 )
  {
  }

  // Copy Constructor
  priv( priv const &other )
    : n_features( other.n_features )
    , n_octave_layers( other.n_octave_layers )
    , contrast_threshold( other.contrast_threshold )
    , edge_threshold( other.edge_threshold )
    , sigma( other.sigma )
  {
  }

  // Create new algorithm instance from current parameters
  cv::Ptr<cv_SIFT_t> create() const
  {
#ifndef MAPTK_HAS_OPENCV_VER_3
    return cv::Ptr<cv_SIFT_t>(
      new cv_SIFT_t( n_features, n_octave_layers, contrast_threshold,
                     edge_threshold, sigma )
    );
#else
    return cv_SIFT_t::create( n_features, n_octave_layers, contrast_threshold,
                              edge_threshold, sigma );
#endif
  }

#ifndef MAPTK_HAS_OPENCV_VER_3
  // Update algorithm with current parameter, 2.4.x only
  void update( cv::Ptr<cv_SIFT_t> a ) const
  {
    a->set( "nFeatures", n_features );
    a->set( "nOctaveLayers", n_octave_layers );
    a->set( "contrastThreshold", contrast_threshold );
    a->set( "edgeThreshold", edge_threshold );
    a->set( "sigma", sigma );
  }
#endif

  // Update config block with current parameter values
  void update_config( config_block_sptr config ) const
  {
    config->set_value( "n_features", n_features,
                       "The number of best features to retain. The features "
                       "are ranked by their scores (measured in SIFT algorithm "
                       "as the local contrast" );
    config->set_value( "n_octave_layers", n_octave_layers,
                       "The number of layers in each octave. 3 is the value "
                       "used in D. Lowe paper. The number of octaves is "
                       "computed automatically from the image resolution." );
    config->set_value( "contrast_threshold", contrast_threshold,
                       "The contrast threshold used to filter out weak "
                       "features in semi-uniform (low-contrast) regions. The "
                       "larger the threshold, the less features are produced "
                       "by the detector." );
    config->set_value( "edge_threshold", edge_threshold,
                       "The threshold used to filter out edge-like features. "
                       "Note that the its meaning is different from the "
                       "contrast_threshold, i.e. the larger the "
                       "edge_threshold, the less features are filtered out "
                       "(more features are retained)." );
    config->set_value( "sigma", sigma,
                       "The sigma of the Gaussian applied to the input image "
                       "at the octave #0. If your image is captured with a "
                       "weak camera with soft lenses, you might want to reduce "
                       "the number." );
  }

  // Set current values based on config block
  void set_config( config_block_sptr config )
  {
    n_features = config->get_value<int>( "n_features" );
    n_octave_layers = config->get_value<int>( "n_octave_layers");
    contrast_threshold = config->get_value<double>( "contrast_threshold" );
    edge_threshold = config->get_value<double>( "edge_threshold" );
    sigma = config->get_value<double>( "sigma" );
  }

  // Parameters
  int n_features;
  int n_octave_layers;
  double contrast_threshold;
  double edge_threshold;
  double sigma;
};

} // end namespace anonymous


class detect_features_SIFT::priv
  : public ocv::priv
{
};


class extract_descriptors_SIFT::priv
  : public ocv::priv
{
};


detect_features_SIFT
::detect_features_SIFT()
  : p_( new priv )
{
  attach_logger("maptk.ocv.SIFT");
  detector = p_->create();
}


detect_features_SIFT
::detect_features_SIFT(detect_features_SIFT const &other)
  : p_( new priv( *other.p_ ) )
{
  attach_logger("maptk.ocv.SIFT");
  detector = p_->create();
}


detect_features_SIFT
::~detect_features_SIFT()
{
}


vital::config_block_sptr
detect_features_SIFT
::get_configuration() const
{
  config_block_sptr config = detect_features::get_configuration();
  p_->update_config( config );
  return config;
}


void
detect_features_SIFT
::set_configuration(vital::config_block_sptr config)
{
  config_block_sptr c = get_configuration();
  c->merge_config( config );
  p_->set_config( c );

#ifndef MAPTK_HAS_OPENCV_VER_3
  p_->update( detector );
#else
  // version 3.x doesn't have parameter update methods
  detector = p_->create();
#endif
}


bool
detect_features_SIFT
::check_configuration(vital::config_block_sptr config) const
{
  return true;
}


extract_descriptors_SIFT
::extract_descriptors_SIFT()
  : p_( new priv )
{
  attach_logger("maptk.ocv.SIFT");
  extractor = p_->create();
}


extract_descriptors_SIFT
::extract_descriptors_SIFT(extract_descriptors_SIFT const &other)
  : p_( new priv( *other.p_ ) )
{
  attach_logger("maptk.ocv.SIFT");
  extractor = p_->create();
}


extract_descriptors_SIFT
::~extract_descriptors_SIFT()
{
}


vital::config_block_sptr
extract_descriptors_SIFT
::get_configuration() const
{
  config_block_sptr config = extract_descriptors::get_configuration();
  p_->update_config(config);
  return config;
}


void
extract_descriptors_SIFT
::set_configuration(vital::config_block_sptr config)
{
  config_block_sptr c = get_configuration();
  c->merge_config(config);
  p_->set_config(c);
#ifndef MAPTK_HAS_OPENCV_VER_3
  p_->update( extractor );
#else
  extractor = p_->create();
#endif
}


bool
extract_descriptors_SIFT
::check_configuration(vital::config_block_sptr config) const
{
  return true;
}


} // end namespace ocv
} // end namespace maptk
} // end namespace kwiver

#endif //defined(HAVE_OPENCV_NONFREE) || defined(HAVE_OPENCV_XFEATURES2D)
