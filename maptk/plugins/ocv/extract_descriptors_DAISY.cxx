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
 * \brief OCV DAISY descriptor extractor wrapper implementation
 */

#include "extract_descriptors_DAISY.h"

#ifdef HAVE_OPENCV_XFEATURES2D

#include <opencv2/xfeatures2d.hpp>

using namespace kwiver::vital;

namespace kwiver {
namespace maptk {
namespace ocv {


namespace {

std::string list_norm_options()
{
  std::stringstream ss;
  ss << "\tNRM_NONE    = " << cv::xfeatures2d::DAISY::NRM_NONE << "\n"
     << "\tNRM_PARTIAL = " << cv::xfeatures2d::DAISY::NRM_PARTIAL << "\n"
     << "\tNRM_FULL    = " << cv::xfeatures2d::DAISY::NRM_FULL << "\n"
     << "\tNRM_SIFT    = " << cv::xfeatures2d::DAISY::NRM_SIFT;
  return ss.str();
}

bool check_norm_type( int norm )
{
  switch( norm )
  {
    case cv::xfeatures2d::DAISY::NRM_NONE:
    case cv::xfeatures2d::DAISY::NRM_PARTIAL:
    case cv::xfeatures2d::DAISY::NRM_FULL:
    case cv::xfeatures2d::DAISY::NRM_SIFT:
      return true;
    default:
      return false;
  }
}

} //end namespace anonymous


class extract_descriptors_DAISY::priv
{
public:
  priv()
    : radius( 15 )
    , q_radius( 3 )
    , q_theta( 3 )
    , q_hist( 8 )
    , norm( cv::xfeatures2d::DAISY::NRM_NONE )
    , interpolation( true )
    , use_orientation( false )
  {
  }

  priv( priv const &other )
    : radius( other.radius )
    , q_radius( other.q_radius )
    , q_theta( other.q_theta )
    , q_hist( other.q_hist )
    , norm( other.norm )
    , interpolation( other.interpolation )
    , use_orientation( other.use_orientation )
  {
  }

  cv::Ptr<cv::xfeatures2d::DAISY> create() const
  {
    // TODO: Allow custom homography matrix?
    return cv::xfeatures2d::DAISY::create( radius, q_radius, q_theta, q_hist,
                                           norm, cv::noArray(), interpolation,
                                           use_orientation );
  }

  void update_config( config_block_sptr config ) const
  {
    config->set_value( "radius", radius,
                       "radius of the descriptor at the initial scale" );
    config->set_value( "q_radius", q_radius,
                       "amount of radial range division quantity" );
    config->set_value( "q_theta", q_theta,
                       "amount of angular range division quantity" );
    config->set_value( "q_hist", q_hist,
                       "amount of gradient orientations range division quantity" );
    config->set_value( "norm", norm,
                       "descriptor normalization type. valid choices:\n"
                       + list_norm_options() );
    config->set_value( "interpolation", interpolation,
                       "" );
    config->set_value( "use_orientation", use_orientation,
                       "" );
  }

  void set_config( config_block_sptr config )
  {
    radius = config->get_value<float>( "radius" );
    q_radius = config->get_value<int>( "q_radius" );
    q_theta = config->get_value<int>( "q_theta" );
    q_hist = config->get_value<int>( "q_hist" );
    norm = config->get_value<int>( "norm" );
    interpolation = config->get_value<bool>( "interpolation" );
    use_orientation = config->get_value<bool>( "use_orientation" );
  }

  bool check_config( config_block_sptr config, logger_handle_t const &log ) const
  {
    bool valid = true;

    int norm = config->get_value<int>( "norm" );
    if( ! check_norm_type( norm ) )
    {
      LOG_ERROR( log, "Invalid norm option '" << norm << "'. Valid choices "
                      "are: " << list_norm_options() );
      valid = false;
    }

    return valid;
  }

  // Parameters
  float radius;
  int q_radius;
  int q_theta;
  int q_hist;
  int norm;
  bool interpolation;
  bool use_orientation;
};


extract_descriptors_DAISY
::extract_descriptors_DAISY()
  : p_( new priv )
{
  attach_logger( "maptk.ocv.DAISY" );
  extractor = p_->create();
}


extract_descriptors_DAISY
::extract_descriptors_DAISY(extract_descriptors_DAISY const &other)
  : p_( new priv( *other.p_ ) )
{
  attach_logger( "maptk.ocv.DAISY" );
  extractor = p_->create();
}


extract_descriptors_DAISY
::~extract_descriptors_DAISY()
{
}

vital::config_block_sptr
extract_descriptors_DAISY
::get_configuration() const
{
  config_block_sptr config = ocv::extract_descriptors::get_configuration();
  p_->update_config( config );
  return config;
}


void extract_descriptors_DAISY
::set_configuration(vital::config_block_sptr config)
{
  config_block_sptr c = get_configuration();
  c->merge_config( config );
  p_->set_config( c );
  extractor = p_->create();
}


bool
extract_descriptors_DAISY
::check_configuration(vital::config_block_sptr config) const
{
  config_block_sptr c = get_configuration();
  c->merge_config( config );
  return p_->check_config( c, m_logger );
}


} // end namespace ocv
} // end namespace maptk
} // end namespace kwiver

#endif //HAVE_OPENCV_XFEATURES2D
