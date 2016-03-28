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
 * \brief OCV MSER feature detector wrapper implementation
 */

#include "detect_features_MSER.h"

namespace kwiver {
namespace maptk {
namespace ocv {

using namespace kwiver::vital;

class detect_features_MSER::priv
{
public:
  /// Constructor
  priv()
    : delta( 5 ),
      min_area( 60 ),
      max_area( 14400 ),
      max_variation( 0.25 ),
      min_diversity( 0.2 ),
      max_evolution( 200 ),
      area_threshold( 1.01 ),
      min_margin( 0.003 ),
      edge_blur_size( 5 )
  {
    #ifdef MAPTK_HAS_OPENCV_VER_3
    pass2only = false;
    #endif
  }

  /// Copy Constructor
  priv( priv const &other )
    : delta( other.delta ),
      min_area( other.min_area ),
      max_area( other.max_area ),
      max_variation( other.max_variation ),
      min_diversity( other.min_diversity ),
      max_evolution( other.max_evolution ),
      area_threshold( other.area_threshold ),
      min_margin( other.min_margin ),
      edge_blur_size( other.edge_blur_size )
  {
    #ifdef MAPTK_HAS_OPENCV_VER_3
    pass2only = other.pass2only;
    #endif
  }

  cv::Ptr<cv::MSER> create() const {
#ifndef MAPTK_HAS_OPENCV_VER_3
    // 2.4.x version constructor
    return cv::Ptr<cv::MSER>(
        new cv::MSER( delta, min_area, max_area, max_variation,
                      min_diversity, max_evolution, area_threshold,
                      min_margin, edge_blur_size )
    );
#else
    cv::Ptr<cv::MSER> p =
        cv::MSER::create( delta, min_area, max_area, max_variation,
                          min_diversity, max_evolution, area_threshold,
                          min_margin, edge_blur_size );
    p->setPass2Only( pass2only );
    return p;
#endif
  }

  // OCV 3.x does not have adequate setter functions for updating all parameters
  // the algorithm was constructed with. So, instead of updating, we'll just
  // create a new cv::MSER instance on parameter update.

  /// Update given config block with currently set parameter values
  void update_config( config_block_sptr config ) const
  {
    config->set_value( "delta", delta,
                       "Compares (size[i] - size[i-delta]) / size[i-delta]" );
    config->set_value( "min_area", min_area,
                       "Prune areas smaller than this" );
    config->set_value( "max_area", max_area,
                       "Prune areas larger than this" );
    config->set_value( "max_variation", max_variation,
                       "Prune areas that have similar size to its children" );
    config->set_value( "min_diversity", min_diversity,
                       "For color images, trace back to cut off MSER with "
                         "diversity less than min_diversity" );
    config->set_value( "max_evolution", max_evolution,
                       "The color images, the evolution steps." );
    config->set_value( "area_threshold", area_threshold,
                       "For color images, the area threshold to cause "
                         "re-initialization" );
    config->set_value( "min_margin", min_margin,
                       "For color images, ignore too-small regions." );
    config->set_value( "edge_blur_size", edge_blur_size,
                       "For color images, the aperture size for edge blur" );
#ifdef MAPTK_HAS_OPENCV_VER_3
    config->set_value( "pass2only", pass2only, "Undocumented" );
#endif
  }

  /// Set parameter values based on given config block
  void set_config( config_block_sptr const &c )
  {
    delta = c->get_value<int>("delta");
    min_area = c->get_value<int>("min_area");
    max_area = c->get_value<int>("max_area");
    max_variation = c->get_value<double>("max_variation");
    min_diversity = c->get_value<double>("min_diversity");
    max_evolution = c->get_value<int>("max_evolution");
    area_threshold = c->get_value<double>("area_threshold");
    min_margin = c->get_value<double>("min_margin");
    edge_blur_size = c->get_value<int>("edge_blur_size");
#ifdef MAPTK_HAS_OPENCV_VER_3
    pass2only = c->get_value<bool>("pass2only");
#endif
  }

  /// Check config parameter values
  bool check_config(vital::config_block_sptr const &c,
                    logger_handle_t const &logger) const
  {
    bool valid = true;

    // checking that area values are >= 0
    if( c->get_value<int>("min_area") < 0 ||
        c->get_value<int>("max_area") < 0 ||
        c->get_value<double>("area_threshold") < 0 )
    {
      LOG_ERROR(logger, "Areas should be at least 0.");
      valid = false;
    }

    return valid;
  }

  /// Parameters
  int delta;
  int min_area;
  int max_area;
  double max_variation;
  double min_diversity;
  int max_evolution;
  double area_threshold;
  double min_margin;
  int edge_blur_size;
#ifdef MAPTK_HAS_OPENCV_VER_3
  bool pass2only;
#endif
};


detect_features_MSER
::detect_features_MSER()
  : p_( new priv )
{
  attach_logger("maptk.ocv.detect_features_FAST");
  detector = p_->create();
}


detect_features_MSER
::detect_features_MSER(detect_features_MSER const &other)
  : p_( new priv( *other.p_ ) )
{
  attach_logger("maptk.ocv.detect_features_FAST");
  detector = p_->create();
}


detect_features_MSER
::~detect_features_MSER()
{
}


vital::config_block_sptr
detect_features_MSER
::get_configuration() const
{
  config_block_sptr config = ocv::detect_features::get_configuration();
  p_->update_config( config );
  return config;
}


void detect_features_MSER
::set_configuration(vital::config_block_sptr config)
{
  config_block_sptr c = get_configuration();
  c->merge_config( config );
  p_->set_config( c );
  detector = p_->create();
}


bool
detect_features_MSER
::check_configuration(vital::config_block_sptr config) const
{
  config_block_sptr c = get_configuration();
  c->merge_config(config);
  return p_->check_config( c, m_logger );
}


} // end namespace ocv
} // end namespace maptk
} // end namespace kwiver
