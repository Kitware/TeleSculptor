/*ckwg +29
 * Copyright 2013-2015 by Kitware, Inc.
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
 * \brief OCV match_features algorithm implementation
 */

#include "match_features.h"

#include <vector>

#include <maptk/plugins/ocv/feature_set.h>
#include <maptk/plugins/ocv/descriptor_set.h>
#include <maptk/plugins/ocv/match_set.h>
#include <maptk/plugins/ocv/ocv_algo_tools.h>

#include <opencv2/features2d/features2d.hpp>

using namespace kwiver::vital;

namespace maptk
{

namespace ocv
{


/// Private implementation class
class match_features::priv
{
public:
  /// Constructor
  priv()
  : matcher(cv::DescriptorMatcher::create("FlannBased")),
    cross_check(true),
    cross_check_knn(1)
  {
  }

  priv(const priv& other)
  : matcher(other.matcher->clone(true)),
    cross_check(true),
    cross_check_knn(1)
  {
  }

  /// Compute simple descriptor matching from descriptors 1 to 2
  void simple_match(const cv::Mat& descriptors1,
                    const cv::Mat& descriptors2,
                    std::vector<cv::DMatch>& matches12)
  {
    matcher->match(descriptors1, descriptors2, matches12);
  }

  /// Compute descriptor matching from 1 to 2 and from 2 to 1.
  /**
   * Only return descriptor matches if the one of the top N
   * matches from 1 to 2 is also a top N match from 2 to 1.
   * Here N is defined by parameter cross_check_knn
   */
  void cross_check_match(const cv::Mat& descriptors1,
                         const cv::Mat& descriptors2,
                         std::vector<cv::DMatch>& filtered_matches12)
  {
    filtered_matches12.clear();
    std::vector<std::vector<cv::DMatch> > matches12, matches21;
    matcher->knnMatch( descriptors1, descriptors2,
                       matches12, cross_check_knn );
    matcher->knnMatch( descriptors2, descriptors1,
                       matches21, cross_check_knn );
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

  /// the descriptor matcher algorithm
  cv::Ptr<cv::DescriptorMatcher> matcher;
  bool cross_check;
  unsigned cross_check_knn;
};


/// Constructor
match_features
::match_features()
: d_(new priv)
{
}


/// Copy Constructor
match_features
::match_features(const match_features& other)
: d_(new priv(*other.d_))
{
}


/// Destructor
match_features
::~match_features()
{
}


/// Get this algorithm's \link kwiver::config_block configuration block \endlink
kwiver::config_block_sptr
match_features
::get_configuration() const
{
  kwiver::config_block_sptr config = algorithm::get_configuration();

  get_nested_ocv_algo_configuration("matcher", config, d_->matcher);

  return config;
}


/// Set this algorithm's properties via a config block
void
match_features
::set_configuration(kwiver::config_block_sptr config)
{
  set_nested_ocv_algo_configuration(
      "matcher", config, d_->matcher);
}


/// Check that the algorithm's configuration kwiver::config_block is valid
bool
match_features
::check_configuration(kwiver::config_block_sptr config) const
{
  bool nested_ok =
    check_nested_ocv_algo_configuration<cv::DescriptorMatcher>(
        "matcher", config);

  return nested_ok;
}


/// Match one set of features and corresponding descriptors to another
kwiver::vital::match_set_sptr
match_features
::match(kwiver::vital::feature_set_sptr feat1, kwiver::vital::descriptor_set_sptr desc1,
        kwiver::vital::feature_set_sptr feat2, kwiver::vital::descriptor_set_sptr desc2) const
{
  // Return empty match set pointer if either of the input sets were empty
  // pointers
  if( !desc1 || !desc2 )
  {
    return kwiver::vital::match_set_sptr();
  }
  // Only perform matching if both pointers are valid and if both descriptor
  // sets contain non-zero elements
  if( !desc1->size() || !desc2->size() )
  {
    return kwiver::vital::match_set_sptr( new maptk::ocv::match_set() );
  }

  cv::Mat d1 = descriptors_to_ocv_matrix(*desc1);
  cv::Mat d2 = descriptors_to_ocv_matrix(*desc2);
  std::vector<cv::DMatch> matches;
  if( d_->cross_check )
  {
    d_->cross_check_match(d1, d2, matches);
  }
  else
  {
    d_->simple_match(d1, d2, matches);
  }
  return kwiver::vital::match_set_sptr(new maptk::ocv::match_set(matches));
}


} // end namespace ocv

} // end namespace maptk
