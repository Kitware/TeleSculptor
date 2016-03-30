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
 * \brief OCV AGAST feature detector wrapper
 */

#include "detect_features_AGAST.h"

// Only available in OpenCV 3.x
#ifdef MAPTK_HAS_OPENCV_VER_3

using namespace kwiver::vital;

namespace kwiver {
namespace maptk {
namespace ocv {


namespace {

/**
 * Return multi-line, tabbed list string of available enum types and their values
 */
std::string list_agast_types()
{
  std::stringstream ss;
  ss << "\tAGAST_5_8 = " << cv::AgastFeatureDetector::AGAST_5_8 << "\n"
     << "\tAGAST_7_12d = " << cv::AgastFeatureDetector::AGAST_7_12d << "\n"
     << "\tAGAST_7_12s = " << cv::AgastFeatureDetector::AGAST_7_12s << "\n"
     << "\tOAST_9_16 = " << cv::AgastFeatureDetector::OAST_9_16;
  return ss.str();
}

/**
 * Check that the given integer is one of the valid enum values
 */
bool check_agast_type( int const &type )
{
  switch( type )
  {
    case cv::AgastFeatureDetector::AGAST_5_8:
    case cv::AgastFeatureDetector::AGAST_7_12d:
    case cv::AgastFeatureDetector::AGAST_7_12s:
    case cv::AgastFeatureDetector::OAST_9_16:
      return true;
    default:
      return false;
  }
}

} // end namespace anonymous


class detect_features_AGAST::priv
{
public:
  /// Constructor
  priv()
    : threshold( 10 ),
      nonmax_suppression( true ),
      type( cv::AgastFeatureDetector::OAST_9_16 )
  {
  }

  /// Copy Constructor
  priv( priv const &other )
    : threshold( other.threshold ),
      nonmax_suppression( other.nonmax_suppression ),
      type( other.type )
  {
  }

  /// Create algorithm instance
  cv::Ptr<cv::AgastFeatureDetector> create() const
  {
    return cv::AgastFeatureDetector::create( threshold, nonmax_suppression,
                                             type );
  }

  /// Update given algo parameters with currently set values
  void update(cv::Ptr<cv::AgastFeatureDetector> algo) const
  {
    algo->setThreshold( threshold );
    algo->setNonmaxSuppression( nonmax_suppression );
    algo->setType( type );
  }

  /// Update given config block with currently set parameter values
  void update_config( config_block_sptr config ) const
  {
    config->set_value( "threshold", threshold,
                       "Integer threshold on difference between intensity of "
                       "the central pixel and pixels of a circle around this "
                       "pixel" );
    config->set_value( "nonmax_suppression", nonmax_suppression,
                       "if true, non-maximum suppression is applied to "
                       "detected corners (keypoints)" );
    config->set_value( "type", type,
                       "Neighborhood pattern type. Should be one of the "
                       "following enumeration type values:\n"
                       + list_agast_types() + " (default)" );
  }

  /// Set parameter values based on given config block
  void set_config( config_block_sptr const &config )
  {
    threshold = config->get_value<int>( "threshold" );
    nonmax_suppression = config->get_value<bool>( "nonmax_suppression" );
    type = config->get_value<int>( "type" );
  }

  /// Check config parameter values
  bool check_config(vital::config_block_sptr const &config,
                    logger_handle_t const &logger) const
  {
    bool valid = true;

    int t = config->get_value<int>( "type" );
    if( ! check_agast_type( t ) )
    {
      LOG_ERROR(logger, "Given AGAST type not valid. Must be one of:\n"
                        + list_agast_types() );
      valid = false;
    }

    return valid;
  }

  // Parameters
  int threshold;
  bool nonmax_suppression;
  int type;
};


detect_features_AGAST
::detect_features_AGAST()
  : p_( new priv )
{
  attach_logger( "maptk.ocv.AGAST" );
  detector = p_->create();
}


detect_features_AGAST
::detect_features_AGAST(const detect_features_AGAST &other)
  : p_( new priv( *other.p_ ) )
{
 attach_logger( "maptk.ocv.AGAST" );
  detector = p_->create();
}


detect_features_AGAST
::~detect_features_AGAST()
{
}


vital::config_block_sptr
detect_features_AGAST
::get_configuration() const
{
  config_block_sptr config = ocv::detect_features::get_configuration();
  p_->update_config( config );
  return config;
}


void
detect_features_AGAST
::set_configuration(vital::config_block_sptr config)
{
  config_block_sptr c = get_configuration();
  c->merge_config( config );
  p_->set_config( c );
  p_->update( detector.dynamicCast<cv::AgastFeatureDetector>() );
}


bool
detect_features_AGAST
::check_configuration(vital::config_block_sptr config) const
{
  config_block_sptr c = get_configuration();
  c->merge_config( config );
  return p_->check_config( c, m_logger );
}


} // end namespace kwiver
} // end namespace maptk
} // end namespace ocv

#endif //MAPTK_HAS_OPENCV_VER_3
