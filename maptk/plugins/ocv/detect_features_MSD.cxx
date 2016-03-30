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
 * \brief OCV MSD feature detector wrapper
 */

#include "detect_features_MSD.h"

// Only available in OpenCV 3.x xfeatures2d
#ifdef HAVE_OPENCV_XFEATURES2D

#include <opencv2/xfeatures2d.hpp>

using namespace kwiver::vital;

namespace kwiver {
namespace maptk {
namespace ocv {


class detect_features_MSD::priv
{
public:
  /// Constructor
  priv()
    : patch_radius( 3 )
    , search_area_radius( 5 )
    , nms_radius( 5 )
    , nms_scale_radius( 0 )
    , th_saliency( 250.0f )
    , knn( 4 )
    , scale_factor( 1.25f )
    , n_scales( -1 )
    , compute_orientation( false )
  {
  }

  /// Copy Constructor
  priv( priv const &other )
    : patch_radius( other.patch_radius )
    , search_area_radius( other.search_area_radius )
    , nms_radius( other.nms_radius )
    , nms_scale_radius( other.nms_scale_radius )
    , th_saliency( other.th_saliency )
    , knn( other.knn )
    , scale_factor( other.scale_factor )
    , n_scales( other.n_scales )
    , compute_orientation( other.compute_orientation )
  {
  }

  /// Create algorithm instance
  cv::Ptr<cv::xfeatures2d::MSDDetector> create() const
  {
    return cv::xfeatures2d::MSDDetector::create(
      patch_radius, search_area_radius, nms_radius, nms_scale_radius,
      th_saliency, knn, scale_factor, n_scales, compute_orientation
    );
  }

  /// Update given config block with currently set parameter values
  void update_config( config_block_sptr config ) const
  {
    config->set_value( "patch_radius", patch_radius );
    config->set_value( "search_area_radius", search_area_radius );
    config->set_value( "nms_radius", nms_radius );
    config->set_value( "nms_scale_radius", nms_scale_radius );
    config->set_value( "th_saliency", th_saliency );
    config->set_value( "knn", knn );
    config->set_value( "scale_factor", scale_factor );
    config->set_value( "n_scales", n_scales );
    config->set_value( "compute_orientation", compute_orientation );
  }

  /// Set parameter values based on given config block
  void set_config( config_block_sptr const &config )
  {
    patch_radius = config->get_value<int>( "patch_radius" );
    search_area_radius = config->get_value<int>( "search_area_radius" );
    nms_radius = config->get_value<int>( "nms_radius" );
    nms_scale_radius = config->get_value<int>( "nms_scale_radius" );
    th_saliency = config->get_value<float>( "th_saliency" );
    knn = config->get_value<int>( "knn" );
    scale_factor = config->get_value<float>( "scale_factor" );
    n_scales = config->get_value<int>( "n_scales" );
    compute_orientation = config->get_value<bool>( "compute_orientation" );
  }

  /// Check config parameter values
  bool check_config( vital::config_block_sptr const &config,
                     logger_handle_t const &logger ) const
  {
    return true;
  }

  // Parameters
  int patch_radius;
  int search_area_radius;
  int nms_radius;
  int nms_scale_radius;
  float th_saliency;
  int knn;
  float scale_factor;
  int n_scales;
  bool compute_orientation;
};


detect_features_MSD
::detect_features_MSD()
  : p_( new priv )
{
  attach_logger( "maptk.ocv.MSD" );
  detector = p_->create();
}


detect_features_MSD
::detect_features_MSD(const detect_features_MSD &other)
  : p_( new priv( *other.p_ ) )
{
  attach_logger( "maptk.ocv.MSD" );
  detector = p_->create();
}


detect_features_MSD
::~detect_features_MSD()
{
}


vital::config_block_sptr
detect_features_MSD
::get_configuration() const
{
  config_block_sptr config = ocv::detect_features::get_configuration();
  p_->update_config( config );
  return config;
}


void
detect_features_MSD
::set_configuration(vital::config_block_sptr config)
{
  config_block_sptr c = get_configuration();
  c->merge_config( config );
  p_->set_config( c );
  detector = p_->create();
}


bool
detect_features_MSD
::check_configuration(vital::config_block_sptr config) const
{
  config_block_sptr c = get_configuration();
  c->merge_config( config );
  return p_->check_config( c, m_logger );
}


} // end namespace kwiver
} // end namespace maptk
} // end namespace ocv

#endif //HAVE_OPENCV_XFEATURES2D
