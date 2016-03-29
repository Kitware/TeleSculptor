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
 * \brief OCV LUCID descriptor extractor wrapper implementation
 */

#include "extract_descriptors_LUCID.h"

#ifdef HAVE_OPENCV_XFEATURES2D

#include <opencv2/xfeatures2d.hpp>

using namespace kwiver::vital;

namespace kwiver {
namespace maptk {
namespace ocv {


class extract_descriptors_LUCID::priv
{
public:
  priv()
    : lucid_kernel( 1 )
    , blur_kernel( 1 )
  {
  }

  priv( priv const &other )
    : lucid_kernel( other.lucid_kernel )
    , blur_kernel( other.blur_kernel )
  {
  }

  cv::Ptr<cv::xfeatures2d::LUCID> create() const
  {
    return cv::xfeatures2d::LUCID::create( lucid_kernel, blur_kernel );
  }

  void update_config( config_block_sptr config ) const
  {
    config->set_value( "lucid_kernel", lucid_kernel,
                       "kernel for descriptor construction, where 1=3x3, "
                       "2=5x5, 3=7x7 and so forth" );
    config->set_value( "blur_kernel", blur_kernel,
                       "kernel for blurring image prior to descriptor "
                       "construction, where 1=3x3, 2=5x5, 3=7x7 and so forth" );
  }

  void set_config( config_block_sptr config )
  {
    lucid_kernel = config->get_value<int>( "lucid_kernel" );
    blur_kernel = config->get_value<int>( "blur_kernel" );
  }

  // Parameters
  int lucid_kernel;
  int blur_kernel;
};


extract_descriptors_LUCID
::extract_descriptors_LUCID()
  : p_( new priv )
{
  attach_logger( "maptk.ocv.LUCID" );
  extractor = p_->create();
}


extract_descriptors_LUCID
::extract_descriptors_LUCID(extract_descriptors_LUCID const &other)
  : p_( new priv( *other.p_ ) )
{
  attach_logger( "maptk.ocv.LUCID" );
  extractor = p_->create();
}


extract_descriptors_LUCID
::~extract_descriptors_LUCID()
{
}

vital::config_block_sptr
extract_descriptors_LUCID
::get_configuration() const
{
  config_block_sptr config = ocv::extract_descriptors::get_configuration();
  p_->update_config( config );
  return config;
}


void extract_descriptors_LUCID
::set_configuration(vital::config_block_sptr config)
{
  config_block_sptr c = get_configuration();
  c->merge_config( config );
  p_->set_config( c );
  extractor = p_->create();
}


bool
extract_descriptors_LUCID
::check_configuration(vital::config_block_sptr config) const
{
  return true;
}


} // end namespace ocv
} // end namespace maptk
} // end namespace kwiver

#endif
