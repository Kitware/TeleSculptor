/*ckwg +5
 * Copyright 2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include <maptk/core/algo/match_features_homography.h>
#include <maptk/core/match_set.h>


namespace maptk
{

namespace algo
{


/// Match one set of features and corresponding descriptors to another
match_set_sptr
match_features_homography
::match(feature_set_sptr feat1, descriptor_set_sptr desc1,
        feature_set_sptr feat2, descriptor_set_sptr desc2) const
{
  if( !matcher_ || !h_estimator_ )
  {
    return match_set_sptr();
  }

  // compute the initial matches
  match_set_sptr init_matches = matcher_->match(feat1, desc1, feat2, desc2);

  // estimate a homography from the initial matches
  std::vector<bool> inliers;
  matrix_3x3d H = h_estimator_->estimate(feat1, feat2, init_matches,
                                         inliers, 2.0);
  std::cout << "inlier ratio: "<< std::count(inliers.begin(), inliers.end(), true)
            << "/"<<inliers.size() << std::endl;

  std::vector<maptk::match> m = init_matches->matches();
  std::vector<maptk::match> inlier_m;
  for( unsigned int i=0; i<inliers.size(); ++i )
  {
    if( inliers[i] )
    {
      inlier_m.push_back(m[i]);
    }
  }

  return match_set_sptr(new simple_match_set(inlier_m));
}


} // end namespace algo

} // end namespace maptk
