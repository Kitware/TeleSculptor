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
 * \brief OCV LATCH descriptor extractor wrapper implementation
 */

#include "extract_descriptors_LATCH.h"

#ifdef HAVE_OPENCV_XFEATURES2D

#include <opencv2/xfeatures2d.hpp>

using namespace kwiver::vital;

namespace kwiver {
namespace maptk {
namespace ocv {


class extract_descriptors_LATCH::priv
{
public:
  priv()
    : bytes( 32 )
    , rotation_invariance( true )
    , half_ssd_size( 3 )
  {
  }

  priv( priv const &other )
    : bytes( other.bytes )
    , rotation_invariance( other.rotation_invariance )
    , half_ssd_size( other.half_ssd_size )
  {
  }

  cv::Ptr<cv::xfeatures2d::LATCH> create() const
  {
    return cv::xfeatures2d::LATCH::create( bytes, rotation_invariance,
                                           half_ssd_size );
  }

  void update_config( config_block_sptr config ) const
  {
    config->set_value( "bytes", bytes, "" );
    config->set_value( "rotation_invariance", rotation_invariance, "" );
    config->set_value( "half_ssd_size", half_ssd_size, "" );
  }

  void set_config( config_block_sptr config )
  {
    bytes = config->get_value<int>( "bytes" );
    rotation_invariance = config->get_value<bool>( "rotation_invariance" );
    half_ssd_size = config->get_value<int>( "half_ssd_size" );
  }

  // Parameters
  int bytes;
  bool rotation_invariance;
  int half_ssd_size;
};


extract_descriptors_LATCH
::extract_descriptors_LATCH()
  : p_( new priv )
{
  attach_logger( "maptk.ocv.LATCH" );
  extractor = p_->create();
}


extract_descriptors_LATCH
::extract_descriptors_LATCH(extract_descriptors_LATCH const &other)
  : p_( new priv( *other.p_ ) )
{
  attach_logger( "maptk.ocv.LATCH" );
  extractor = p_->create();
}


extract_descriptors_LATCH
::~extract_descriptors_LATCH()
{
}

vital::config_block_sptr
extract_descriptors_LATCH
::get_configuration() const
{
  config_block_sptr config = ocv::extract_descriptors::get_configuration();
  p_->update_config( config );
  return config;
}


void extract_descriptors_LATCH
::set_configuration(vital::config_block_sptr config)
{
  config_block_sptr c = get_configuration();
  c->merge_config( config );
  p_->set_config( c );
  extractor = p_->create();
}


bool
extract_descriptors_LATCH
::check_configuration(vital::config_block_sptr config) const
{
  return true;
}


} // end namespace ocv
} // end namespace maptk
} // end namespace kwiver

#endif //HAVE_OPENCV_XFEATURES2D
