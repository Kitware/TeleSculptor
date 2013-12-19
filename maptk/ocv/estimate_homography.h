/*ckwg +5
 * Copyright 2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_OCV_ESTIMATE_HOMOGRAPHY_H_
#define MAPTK_OCV_ESTIMATE_HOMOGRAPHY_H_


#include <maptk/core/algo/estimate_homography.h>

namespace maptk
{

namespace ocv
{

/// A class that using OpenCV to estimate a homography from matching 2D points
class estimate_homography
: public algo::algorithm_impl<estimate_homography, algo::estimate_homography>
{
public:
  /// Return the name of this implementation
  std::string impl_name() const { return "ocv"; }

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
           double inlier_scale = 1.0) const;

};


} // end namespace ocv

} // end namespace maptk


#endif // MAPTK_OCV_ESTIMATE_HOMOGRAPHY_H_
