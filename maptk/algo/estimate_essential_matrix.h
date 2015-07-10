/*ckwg +29
 * Copyright 2014-2015 by Kitware, Inc.
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
 * \brief estimate_essential_matrix algorithm definition
 */

#ifndef MAPTK_ALGO_ESTIMATE_ESSENTIAL_MATRIX_H_
#define MAPTK_ALGO_ESTIMATE_ESSENTIAL_MATRIX_H_

#include <maptk/config.h>

#include <vector>

#include <boost/shared_ptr.hpp>

#include <vital/algorithm.h>
#include <vital/feature_set.h>
#include <vital/match_set.h>
#include <vital/matrix.h>
#include <vital/camera_intrinsics.h>

namespace maptk
{

namespace algo
{

/// An abstract base class for estimating an essential matrix from matching 2D points
class MAPTK_LIB_EXPORT estimate_essential_matrix
  : public kwiver::vital::algorithm_def<estimate_essential_matrix>
{
public:
  /// Return the name of this algorithm
  static std::string static_type_name() { return "estimate_essential_matrix"; }

  /// Estimate an essential matrix from corresponding features
  /**
   * \param [in]  feat1 the set of all features from the first image
   * \param [in]  feat2 the set of all features from the second image
   * \param [in]  matches the set of correspondences between \a feat1 and \a feat2
   * \param [in]  cal1 the intrinsic parameters of the first camera
   * \param [in]  cal2 the intrinsic parameters of the second camera
   * \param [out] inliers for each point pair, the value is true if
   *                      this pair is an inlier to the estimate
   * \param [in]  inlier_scale error distance tolerated for matches to be inliers
   */
  virtual
  kwiver::vital::matrix_3x3d
  estimate(kwiver::vital::feature_set_sptr feat1,
           kwiver::vital::feature_set_sptr feat2,
           kwiver::vital::match_set_sptr matches,
           const kwiver::vital::camera_intrinsics_d &cal1,
           const kwiver::vital::camera_intrinsics_d &cal2,
           std::vector<bool>& inliers,
           double inlier_scale = 1.0) const;

  /// Estimate an essential matrix from corresponding features
  /**
   * \param [in]  feat1 the set of all features from the first image
   * \param [in]  feat2 the set of all features from the second image
   * \param [in]  matches the set of correspondences between \a feat1 and \a feat2
   * \param [in]  cal the intrinsic parameters, same for both cameras
   * \param [out] inliers for each point pair, the value is true if
   *                      this pair is an inlier to the estimate
   * \param [in]  inlier_scale error distance tolerated for matches to be inliers
   */
  virtual
  kwiver::vital::matrix_3x3d
  estimate(kwiver::vital::feature_set_sptr feat1,
           kwiver::vital::feature_set_sptr feat2,
           kwiver::vital::match_set_sptr matches,
           const kwiver::vital::camera_intrinsics_d &cal,
           std::vector<bool>& inliers,
           double inlier_scale = 1.0) const;

  /// Estimate an essential matrix from corresponding points
  /**
   * \param [in]  pts1 the vector or corresponding points from the first image
   * \param [in]  pts2 the vector of corresponding points from the second image
   * \param [in]  cal the intrinsic parameters, same for both cameras
   * \param [out] inliers for each point pair, the value is true if
   *                      this pair is an inlier to the estimate
   * \param [in]  inlier_scale error distance tolerated for matches to be inliers
   */
  virtual
  kwiver::vital::matrix_3x3d
  estimate(const std::vector<kwiver::vital::vector_2d>& pts1,
           const std::vector<kwiver::vital::vector_2d>& pts2,
           const kwiver::vital::camera_intrinsics_d &cal,
           std::vector<bool>& inliers,
           double inlier_scale = 1.0) const;

  /// Estimate an essential matrix from corresponding points
  /**
   * \param [in]  pts1 the vector or corresponding points from the first image
   * \param [in]  pts2 the vector of corresponding points from the second image
   * \param [in]  cal1 the intrinsic parameters of the first camera
   * \param [in]  cal2 the intrinsic parameters of the second camera
   * \param [out] inliers for each point pa:wir, the value is true if
   *                      this pair is an inlier to the estimate
   * \param [in]  inlier_scale error distance tolerated for matches to be inliers
   */
  virtual
  kwiver::vital::matrix_3x3d
  estimate(const std::vector<kwiver::vital::vector_2d>& pts1,
           const std::vector<kwiver::vital::vector_2d>& pts2,
           const kwiver::vital::camera_intrinsics_d &cal1,
           const kwiver::vital::camera_intrinsics_d &cal2,
           std::vector<bool>& inliers,
           double inlier_scale = 1.0) const = 0;


};


/// Shared pointer type of base estimate_homography algorithm definition class
typedef boost::shared_ptr<estimate_essential_matrix> estimate_essential_matrix_sptr;


} // end namespace algo

} // end namespace maptk


#endif // MAPTK_ALGO_ESTIMATE_ESSENTIAL_MATRIX_H_
