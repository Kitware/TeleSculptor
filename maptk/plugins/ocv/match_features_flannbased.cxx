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
 * \brief OCV flann-based feature matcher wrapper implementation
 */

#include "match_features_flannbased.h"

namespace kwiver {
namespace maptk {
namespace ocv {


class match_features_flannbased::priv
{
public:
  /// Constructor
  priv()
    : cross_check( true ),
      cross_check_k( 1 ),
      matcher( new cv::FlannBasedMatcher )
  {
  }

  /// Copy Constructor
  priv( priv const &other )
    : cross_check( other.cross_check ),
      cross_check_k( other.cross_check_k ),
      matcher( new cv::FlannBasedMatcher )
  {
  }

  // Can't currently update parameters on BF implementation, so no update
  // function. Will need to create a new instance on each parameter update.

  /// Create a new flann-based matcher instance and set our matcher param to it
  void create()
  {
    // cross version compatible
    matcher = cv::Ptr<cv::FlannBasedMatcher>( new cv::FlannBasedMatcher );
  }

  /// Compute descriptor matching from 1 to 2 and from 2 to 1.
  /**
   * Only return descriptor matches if the one of the top N
   * matches from 1 to 2 is also a top N match from 2 to 1.
   * Here N is defined by parameter cross_check_knn
   */
  void cross_check_match(const cv::Mat& descriptors1,
                         const cv::Mat& descriptors2,
                         std::vector<cv::DMatch>& filtered_matches12) const
  {
    filtered_matches12.clear();
    std::vector<std::vector<cv::DMatch> > matches12, matches21;
    matcher->knnMatch( descriptors1, descriptors2,
                       matches12, cross_check_k );
    matcher->knnMatch( descriptors2, descriptors1,
                       matches21, cross_check_k );
    for( size_t m = 0; m < matches12.size(); m++ )
    {
      bool find_cross_check = false;
      for( size_t fk = 0; fk < matches12[m].size(); fk++ )
      {
        cv::DMatch forward = matches12[m][fk];

        for( size_t bk = 0; bk < matches21[forward.trainIdx].size(); bk++ )
        {
          cv::DMatch backward = matches21[forward.trainIdx][bk];
          if( backward.trainIdx == forward.queryIdx )
          {
            filtered_matches12.push_back(forward);
            find_cross_check = true;
            break;
          }
        }
        if( find_cross_check ) break;
      }
    }
  }

  /// Parameters
  bool cross_check;
  unsigned cross_check_k;
  cv::Ptr<cv::FlannBasedMatcher> matcher;

}; // end match_features_flannbased::priv


match_features_flannbased
::match_features_flannbased()
  : p_( new priv )
{
  std::stringstream ss;
  ss << type_name() << "." << impl_name();
  attach_logger( ss.str() );
}


match_features_flannbased
::match_features_flannbased(match_features_flannbased const &other)
  : p_( new priv( *other.p_ ) )
{
  std::stringstream ss;
  ss << type_name() << "." << impl_name();
  attach_logger( ss.str() );
}


match_features_flannbased
::~match_features_flannbased()
{
}


vital::config_block_sptr
match_features_flannbased
::get_configuration() const
{
  vital::config_block_sptr config = match_features::get_configuration();

  config->set_value( "cross_check", p_->cross_check,
                     "If cross-check filtering should be performed." );
  config->set_value( "cross_check_k", p_->cross_check_k,
                     "Number of neighbors to use when cross checking" );

  return config;
}


void
match_features_flannbased
::set_configuration(vital::config_block_sptr in_config)
{
  vital::config_block_sptr config = get_configuration();
  config->merge_config( in_config );

  p_->cross_check = config->get_value<bool>( "cross_check" );
  p_->cross_check_k = config->get_value<unsigned>( "cross_check_k" );

  p_->create();
}


bool
match_features_flannbased
::check_configuration(vital::config_block_sptr in_config) const
{
  vital::config_block_sptr config = get_configuration();
  config->merge_config(in_config);
  bool valid = true;

  unsigned k = config->get_value<unsigned>("cross_check_k");
  if( k == 0 )
  {
    m_logger->log_error("Cross-check K value must be greater than 0.");
    valid = false;
  }

  return valid;
}


void
match_features_flannbased
::ocv_match(const cv::Mat &descriptors1, const cv::Mat &descriptors2,
            std::vector<cv::DMatch> &matches) const
{
  if( p_->cross_check )
  {
    p_->cross_check_match(descriptors1, descriptors2, matches);
  }
  else
  {
    p_->matcher->match(descriptors1, descriptors2, matches);
  }
}


} // end namespace ocv
} // end namespace maptk
} // end namespace kwiver
