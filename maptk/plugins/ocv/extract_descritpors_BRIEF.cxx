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
 * \brief OCV BRIEF descriptor extractor wrapper implementation
 */

#include "extract_descriptors_BRIEF.h"

#if ! defined(MAPTK_HAS_OPENCV_VER_3) || defined(HAVE_OPENCV_XFEATURES2D)

#include <sstream>
#include <opencv2/xfeatures2d.hpp>


#ifndef MAPTK_HAS_OPENCV_VER_3
typedef cv::BriefDescriptorExtractor cv_BRIEF_t;
#else
typedef cv::xfeatures2d::BriefDescriptorExtractor cv_BRIEF_t;
#endif

using namespace kwiver::vital;

namespace kwiver {
namespace maptk {
namespace ocv {


class extract_descriptors_BRIEF::priv
{
public:
  /// Constructor
  priv()
    : bytes( 32 )
#ifdef MAPTK_HAS_OPENCV_VER_3
    , use_orientation( false )
#endif
  {
  }

  /// Copy Constructor
  priv(priv const &other)
    : bytes( other.bytes )
#ifdef MAPTK_HAS_OPENCV_VER_3
    , use_orientation( other.use_orientation )
#endif
  {
  }

  /// Create new algorithm instance using current parameter values
  cv::Ptr<cv_BRIEF_t> create() const
  {
#ifndef MAPTK_HAS_OPENCV_VER_3
    return cv::Ptr<cv_BRIEF_t>( new cv_BRIEF_t(bytes) );
#else
    return cv_BRIEF_t::create( bytes, use_orientation );
#endif
  }

#ifndef MAPTK_HAS_OPENCV_VER_3
  /// Update given algorithm using current parameter values
  void update(cv::Ptr<cv_BRIEF_t> descriptor) const
  {
    descriptor->set( "bytes", bytes );
  }
#endif

  void update_config( config_block_sptr config ) const
  {
    config->set_value( "bytes", bytes,
                       "Length of descriptor in bytes. It can be equal 16, 32 "
                       "or 64 bytes." );
#ifdef MAPTK_HAS_OPENCV_VER_3
    config->set_value( "use_orientation", use_orientation,
                       "sample patterns using keypoints orientation, disabled "
                       "by default." );
#endif
  }

  void set_config( config_block_sptr config )
  {
    bytes = config->get_value<int>( "bytes" );
#ifdef MAPTK_HAS_OPENCV_VER_3
    use_orientation = config->get_value<bool>( "use_orientation" );
#endif
  }

  bool check_config( config_block_sptr config, logger_handle_t const &logger ) const
  {
    bool valid = true;

    // check that bytes param is one of the required 3 values
    int b = config->get_value<int>( "bytes" );
    if( ! ( b == 16 || b == 32 || b == 64 ) )
    {
      LOG_ERROR( logger,
                 "Bytes parameter must be either 16, 32 or 64. Given: " << b );
      valid = false;
    }

    return valid;
  }

  // Parameters
  int bytes;
#ifdef MAPTK_HAS_OPENCV_VER_3
  bool use_orientation;
#endif
};


/// Constructor
extract_descriptors_BRIEF
::extract_descriptors_BRIEF()
  : p_(new priv)
{
  attach_logger("maptk.ocv.BRIEF");
  extractor = p_->create();
}


/// Copy Constructor
extract_descriptors_BRIEF
::extract_descriptors_BRIEF(extract_descriptors_BRIEF const &other)
  : p_( new priv(*other.p_) )
{
  attach_logger("maptk.ocv.BRIEF");
  extractor = p_->create();
}


/// Destructor
extract_descriptors_BRIEF
::~extract_descriptors_BRIEF()
{
}


vital::config_block_sptr
extract_descriptors_BRIEF
::get_configuration() const
{
  vital::config_block_sptr config =
      maptk::ocv::extract_descriptors::get_configuration();
  p_->update_config( config );
  return config;
}


void
extract_descriptors_BRIEF
::set_configuration(vital::config_block_sptr config)
{
  vital::config_block_sptr c = get_configuration();
  c->merge_config( config );
  p_->set_config( c );

#ifndef MAPTK_HAS_OPENCV_VER_3
  p_->update( extractor );
#else
  extractor = p_->create();
#endif
}


bool
extract_descriptors_BRIEF
::check_configuration(vital::config_block_sptr in_config) const
{
  vital::config_block_sptr config = get_configuration();
  config->merge_config(in_config);
  return p_->check_config( config, m_logger );
}


} // end namespace ocv
} // end namespace maptk
} // end namespace kwiver

#endif // has OCV support
