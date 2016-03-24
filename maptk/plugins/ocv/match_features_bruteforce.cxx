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
 * \brief OCV brute-force feature matcher wrapper implementation
 */

#include "match_features_bruteforce.h"

namespace kwiver {
namespace maptk {
namespace ocv {


class match_features_bruteforce::priv
{
public:
  priv(int norm_type=cv::NORM_L2, bool cross_check=false)
      : norm_type( norm_type ),
        cross_check( cross_check ),
        matcher( new cv::BFMatcher(norm_type, cross_check) )
  {
  }

  /// Copy Constructor
  priv( priv const &other )
    : norm_type( other.norm_type ),
      cross_check( other.cross_check ),
      matcher( other.matcher->clone() )
  {
  }

  // Can't currently update parameters on BF implementation, so no update
  // function. Will need to create a new instance on each parameter update.

  /// Create a new brute-force matcher instance and set our matcher param to it
  void create()
  {
    // cross version compatible
    matcher = cv::Ptr<cv::BFMatcher>(
        new cv::BFMatcher(norm_type, cross_check)
    );
  }

  /// Parameters
  int norm_type;
  bool cross_check;
  cv::Ptr<cv::BFMatcher> matcher;

}; // end match_features_bruteforce::priv


namespace {

/// Norm type info string generator
std::string str_list_enum_values()
{
  std::stringstream ss;
  ss << "cv::NORM_INF="       << cv::NORM_INF       << ", "
     << "cv::NORM_L1="        << cv::NORM_L1        << ", "
     << "cv::NORM_L2="        << cv::NORM_L2        << ", "
     << "cv::NORM_L2SQR="     << cv::NORM_L2SQR     << ", "
     << "cv::NORM_HAMMING="   << cv::NORM_HAMMING   << ", "
     << "cv::NORM_HAMMING2="  << cv::NORM_HAMMING2  << ", "
     << "cv::NORM_TYPE_MASK=" << cv::NORM_TYPE_MASK << ", "
     << "cv::NORM_RELATIVE="  << cv::NORM_RELATIVE  << ", "
     << "cv::NORM_MINMAX="    << cv::NORM_MINMAX;
  return ss.str();
}

/// Check value against known OCV norm enum values
bool check_norm_enum_value(int norm_type)
{
  switch( norm_type )
  {
    case cv::NORM_INF:
    case cv::NORM_L1:
    case cv::NORM_L2:
    case cv::NORM_L2SQR:
    case cv::NORM_HAMMING:
    case cv::NORM_HAMMING2:
    //case cv::NORM_TYPE_MASK:  // This is the same value as HAMMING2 apparently
    case cv::NORM_RELATIVE:
    case cv::NORM_MINMAX:
      return true;
    default:
      return false;
  }
}

}


match_features_bruteforce
::match_features_bruteforce()
  : p_( new priv )
{
  std::stringstream ss;
  ss << type_name() << "." << impl_name();
  attach_logger( ss.str() );
}


match_features_bruteforce
::match_features_bruteforce(match_features_bruteforce const &other)
  : p_( new priv( *other.p_ ) )
{
  std::stringstream ss;
  ss << type_name() << "." << impl_name();
  attach_logger( ss.str() );
}


match_features_bruteforce
::~match_features_bruteforce()
{
}


vital::config_block_sptr
match_features_bruteforce
::get_configuration() const
{
  vital::config_block_sptr config = match_features::get_configuration();

  config->set_value( "cross_check", p_->cross_check,
                     "Perform cross checking when finding matches to filter "
                     "through only the consistent pairs. This is an "
                     "alternative to the ratio test used by D. Lowe in the "
                     "SIFT paper." );

  std::stringstream ss;
  ss << "Normalization type enum value. This should be one of the enum values: "
     << str_list_enum_values();
  config->set_value( "norm_type", p_->norm_type, ss.str());

  return config;
}


void
match_features_bruteforce
::set_configuration(vital::config_block_sptr in_config)
{
  vital::config_block_sptr config = get_configuration();
  config->merge_config(in_config);

  p_->cross_check = config->get_value<bool>("cross_check");
  p_->norm_type = config->get_value<int>("norm_type");

  // Create new instance with the updated parameters
  p_->create();
}


bool
match_features_bruteforce
::check_configuration(vital::config_block_sptr config) const
{
  bool valid = true;

  // user has the chance to input an incorret value for the norm type enum value
  int norm_type = config->get_value<int>( "norm_type" );
  if( ! check_norm_enum_value( norm_type ) )
  {
    std::stringstream ss;
    ss << "Incorrect norm type enum value given: '" << norm_type << "'. "
       << "Valid values are: " << str_list_enum_values();
    m_logger->log_error( ss.str() );
    valid = false;
  }

  return valid;
}


void
match_features_bruteforce
::ocv_match(const cv::Mat &descriptors1, const cv::Mat &descriptors2,
            std::vector<cv::DMatch> &matches) const
{
  p_->matcher->match( descriptors1, descriptors2, matches );
}


} // end namespace ocv
} // end namespace maptk
} // end namespace kwiver

