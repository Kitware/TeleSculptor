/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief estimate_homography algorithm definition
 */

#ifndef MAPTK_ALGO_ESTIMATE_HOMOGRAPHY_H_
#define MAPTK_ALGO_ESTIMATE_HOMOGRAPHY_H_

#include <maptk/core/core_config.h>

#include <vector>

#include <boost/shared_ptr.hpp>

#include <maptk/core/algo/algorithm.h>
#include <maptk/core/feature_set.h>
#include <maptk/core/match_set.h>
#include <maptk/core/matrix.h>

namespace maptk
{

namespace algo
{

/// An abstract base class for estimating a homography from matching 2D points
class MAPTK_CORE_EXPORT estimate_homography
  : public algorithm_def<estimate_homography>
{
public:
  /// Return the name of this algorithm
  std::string type_name() const { return "estimate_homography"; }

  /// Estimate a homography matrix from corresponding features
  /**
   * \param [in]  feat1 the set of all features from the source image
   * \param [in]  feat2 the set of all features from the destination image
   * \param [in]  matches the set of correspondences between \a feat1 and \a feat2
   * \param [out] inliers for each match in \a matcher, the value is true if
   *                      this pair is an inlier to the homography estimate
   * \param [in]  inlier_scale error distance tolerated for matches to be inliers
   */
  virtual matrix_3x3d
  estimate(feature_set_sptr feat1,
           feature_set_sptr feat2,
           match_set_sptr matches,
           std::vector<bool>& inliers,
           double inlier_scale = 1.0) const;

  /// Estimate a homography matrix from corresponding points
  /**
   * \param [in]  pts1 the vector or corresponding points from the source image
   * \param [in]  pts2 the vector of corresponding points from the destination image
   * \param [out] inliers for each point pair, the value is true if
   *                      this pair is an inlier to the homography estimate
   * \param [in]  inlier_scale error distance tolerated for matches to be inliers
   */
  virtual matrix_3x3d
  estimate(const std::vector<vector_2d>& pts1,
           const std::vector<vector_2d>& pts2,
           std::vector<bool>& inliers,
           double inlier_scale = 1.0) const = 0;

};


typedef boost::shared_ptr<estimate_homography> estimate_homography_sptr;


} // end namespace algo

} // end namespace maptk


#endif // MAPTK_ALGO_ESTIMATE_HOMOGRAPHY_H_
