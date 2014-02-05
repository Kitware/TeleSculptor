/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include "match_features.h"

#include <vector>
#include <maptk/ocv/feature_set.h>
#include <maptk/ocv/descriptor_set.h>
#include <maptk/ocv/match_set.h>
#include <maptk/ocv/ocv_algo_tools.h>
#include <opencv2/features2d/features2d.hpp>


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


/// Get this algorithm's \link maptk::config_block configuration block \endlink
config_block_sptr
match_features
::get_configuration() const
{
  config_block_sptr config = algorithm::get_configuration();

  get_nested_ocv_algo_configuration(
      "FlannBased_matcher", config, d_->matcher);

  return config;
}


/// Set this algorithm's properties via a config block
void
match_features
::set_configuration(config_block_sptr config)
{
  set_nested_ocv_algo_configuration(
      "FlannBased_matcher", config, d_->matcher);
}


/// Check that the algorithm's configuration config_block is valid
bool
match_features
::check_configuration(config_block_sptr config) const
{
  bool nested_ok = check_nested_ocv_algo_configuration(
      "FlannBased_matcher", config, d_->matcher);

  return nested_ok;
}


/// Match one set of features and corresponding descriptors to another
match_set_sptr
match_features
::match(feature_set_sptr feat1, descriptor_set_sptr desc1,
        feature_set_sptr feat2, descriptor_set_sptr desc2) const
{
  if( !desc1 || !desc2 )
  {
    return match_set_sptr();
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
  return match_set_sptr(new match_set(matches));
}


} // end namespace ocv

} // end namespace maptk
