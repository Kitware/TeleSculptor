/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief estimate_homography algorithm definition instantiation + implementation
 */

#include <maptk/core/algo/estimate_homography.h>
#include <maptk/core/algo/algorithm.txx>
#include <boost/foreach.hpp>

INSTANTIATE_ALGORITHM_DEF(maptk::algo::estimate_homography);


namespace maptk
{

namespace algo
{

/// Estimate a homography matrix from corresponding features
matrix_3x3d
estimate_homography
::estimate(feature_set_sptr feat1,
           feature_set_sptr feat2,
           match_set_sptr matches,
           std::vector<bool>& inliers,
           double inlier_scale) const
{
  std::vector<feature_sptr> vf1 = feat1->features();
  std::vector<feature_sptr> vf2 = feat2->features();
  std::vector<match> mset = matches->matches();
  std::vector<vector_2d> vv1, vv2;
  BOOST_FOREACH(match m, mset)
  {
    vv1.push_back(vf1[m.first]->loc());
    vv2.push_back(vf2[m.second]->loc());
  }
  return this->estimate(vv1, vv2, inliers, inlier_scale);
}

} // end namespace algo

} // end namespace maptk
