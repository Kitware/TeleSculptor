/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief Definition for similarity transform estimation algorithm
 */

#ifndef MAPTK_CORE_ALGO_ESTIMATE_SIMILARITY_TRANSFORM_H
#define MAPTK_CORE_ALGO_ESTIMATE_SIMILARITY_TRANSFORM_H

#include <string>
#include <vector>

#include <maptk/core/algo/algorithm.h>
#include <maptk/core/camera.h>
#include <maptk/core/config_block.h>
#include <maptk/core/core_config.h>
#include <maptk/core/landmark.h>
#include <maptk/core/similarity.h>
#include <maptk/core/vector.h>

namespace maptk
{

namespace algo
{

class MAPTK_CORE_EXPORT estimate_similarity_transform
  : public algorithm_def<estimate_similarity_transform>
{
public:
  /// Name of this algo definition
  std::string type_name() const { return "estimate_similarity_transform"; }

  /// Estimate the similarity transformation between two parallel sets of 3d points
  /**
   * \param from List of length N of 3D points in the from space.
   * \param to   List of length N of 3D points in the to space.
   * \throws algorithm_exception When the from and to points sets are
   *                             misaligned, insufficient or degenerate.
   * \returns An estimated similarity transform mapping 3D points in the
   *          \c from space to points in the \c to space.
   */
  virtual similarity_d
  estimate_transform(std::vector<vector_3d> &from,
                     std::vector<vector_3d> &to) const = 0;

  /// Estimate the similarity transform between two parallel sets of cameras
  /**
   * \param from List of length N of cameras in the from space.
   * \param to   List of length N of cameras in the to space.
   * \throws algorithm_exception When the from and to points sets are
   *                             misaligned, insufficient or degenerate.
   * \returns An estimated similarity transform mapping camera centers in the
   *          \c from space to camera centers in the \c to space.
   */
  virtual similarity_d
  estimate_transform(std::vector<camera_sptr> &from,
                     std::vector<camera_sptr> &to) const;

  /// Estimate the similarity transform between two parallel sets of landmarks.
  /**
   * \param from List of length N of landmarks in the from space.
   * \param to   List of length N of landmarks in the to space.
   * \throws algorithm_exception When the from and to points sets are
   *                             misaligned, insufficient or degenerate.
   * \returns An estinated similarity transform mapping landmark locations in
   *          the \c from space to located in the \c to space.
   */
  virtual similarity_d
  estimate_transform(std::vector<landmark_sptr> &from,
                     std::vector<landmark_sptr> &to) const;
};


/// Shared pointer for similarity transformation algorithms
typedef boost::shared_ptr<estimate_similarity_transform> estimate_similarity_transform_sptr;


} // end algo namespace

} // end maptk namespace

#endif // MAPTK_CORE_ALGO_ESTIMATE_SIMILARITY_TRANSFORM_H
