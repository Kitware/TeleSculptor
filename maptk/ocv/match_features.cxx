/*ckwg +5
 * Copyright 2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include "match_features.h"

#include <vector>
#include <maptk/ocv/feature_set.h>
#include <maptk/ocv/descriptor_set.h>
#include <maptk/ocv/match_set.h>
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
  : matcher(cv::DescriptorMatcher::create("FlannBased"))
  {
  }

  priv(const priv& other)
  : matcher(cv::DescriptorMatcher::create(other.matcher->name()))
  {
  }

  /// the descriptor matcher algorithm
  cv::Ptr<cv::DescriptorMatcher> matcher;
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
  d_->matcher->match(d1, d2, matches);
  return match_set_sptr(new match_set(matches));
}


} // end namespace ocv

} // end namespace maptk
