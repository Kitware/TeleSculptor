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

#ifndef MAPTK_HAS_OPENCV_VER_3

#include "extract_descriptors_BRIEF.h"

#include <sstream>

namespace kwiver {
namespace maptk {
namespace ocv {


class extract_descriptors_BRIEF::priv
{
public:
  /// Constructor
  priv()
    : bytes( 32 )
  {
  }

  /// Copy Constructor
  priv(priv const &other)
    : bytes( other.bytes )
  {
  }

  /// Create new algorithm instance using current parameter values
  cv::Ptr<cv::BriefDescriptorExtractor> create() const
  {
    // BRIEF extractor is only defined in 2.4.9.1 and below, so we'll only use
    // that OpeCV version creation method.
    return cv::Ptr<cv::BriefDescriptorExtractor>(
        new cv::BriefDescriptorExtractor(bytes)
    );
  }

  /// Update given algorithm using current parameter values
  void update(cv::Ptr<cv::BriefDescriptorExtractor> descriptor) const
  {
    descriptor->set("bytes", bytes);
  }

  // Parameters
  int bytes;
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

  config->set_value( "bytes", p_->bytes,
                     "Length of descriptor in bytes. It can be equal 16, 32 or "
                     "64 bytes." );

  return config;
}


void
extract_descriptors_BRIEF
::set_configuration(vital::config_block_sptr in_config)
{
  vital::config_block_sptr config = get_configuration();
  config->merge_config( in_config );

  p_->bytes = config->get_value<int>( "bytes" );

  p_->update( extractor );
}


bool
extract_descriptors_BRIEF
::check_configuration(vital::config_block_sptr in_config) const
{
  vital::config_block_sptr config = get_configuration();
  config->merge_config(in_config);
  bool valid = true;

  // check that bytes param is one of the required 3 values
  int b = config->get_value<int>( "bytes" );
  if( ! ( b == 16 || b == 32 || b == 64 ) )
  {
    std::stringstream ss;
    ss << "Bytes parameter must be either 16, 32 or 64. Given: " << b;
    m_logger->log_error( ss.str() );
    valid = false;
  }

  return valid;
}


} // end namespace ocv
} // end namespace maptk
} // end namespace kwiver

#endif // MAPTK_HAS_OPENCV_VER_3
