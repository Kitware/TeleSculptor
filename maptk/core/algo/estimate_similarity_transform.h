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
#include <maptk/core/camera_map.h>
#include <maptk/core/config_block.h>
#include <maptk/core/core_config.h>
#include <maptk/core/landmark.h>
#include <maptk/core/landmark_map.h>
#include <maptk/core/similarity.h>
#include <maptk/core/vector.h>

namespace maptk
{

namespace algo
{

/// Algorithm for estimating the similarity transform between two point sets
class MAPTK_CORE_EXPORT estimate_similarity_transform
  : public algorithm_def<estimate_similarity_transform>
{
public:
  /// Name of this algo definition
  std::string type_name() const { return "estimate_similarity_transform"; }

  /// Estimate the similarity transform between two corresponding point sets
  /**
   * \param from List of length N of 3D points in the from space.
   * \param to   List of length N of 3D points in the to space.
   * \throws algorithm_exception When the from and to point sets are
   *                             misaligned, insufficient or degenerate.
   * \returns An estimated similarity transform mapping 3D points in the
   *          \c from space to points in the \c to space.
   */
  virtual similarity_d
  estimate_transform(std::vector<vector_3d> const& from,
                     std::vector<vector_3d> const& to) const = 0;

  /// Estimate the similarity transform between two corresponding sets of cameras
  /**
   * \param from List of length N of cameras in the from space.
   * \param to   List of length N of cameras in the to space.
   * \throws algorithm_exception When the from and to point sets are
   *                             misaligned, insufficient or degenerate.
   * \returns An estimated similarity transform mapping camera centers in the
   *          \c from space to camera centers in the \c to space.
   */
  virtual similarity_d
  estimate_transform(std::vector<camera_sptr> const& from,
                     std::vector<camera_sptr> const& to) const;

  /// Estimate the similarity transform between two corresponding sets of landmarks.
  /**
   * \param from List of length N of landmarks in the from space.
   * \param to   List of length N of landmarks in the to space.
   * \throws algorithm_exception When the from and to point sets are
   *                             misaligned, insufficient or degenerate.
   * \returns An estinated similarity transform mapping landmark locations in
   *          the \c from space to located in the \c to space.
   */
  virtual similarity_d
  estimate_transform(std::vector<landmark_sptr> const& from,
                     std::vector<landmark_sptr> const& to) const;

  /// Estimate the similarity transform between two corresponding camera maps
  /**
   * Cameras with corresponding frame IDs in the two maps are paired for
   * transform estimation. Cameras with no corresponding frame ID in the other
   * map are ignored. An algorithm_exception is thrown if there are no shared
   * frame IDs between the two provided maps (nothing to pair).
   *
   * \throws algorithm_exception When the from and to point sets are
   *                             misaligned, insufficient or degenerate.
   * \param from Map of original cameras, sharing N frames with the transformed
   *             cameras, where N > 0.
   * \param to   Map of transformed cameras, sharing N frames with the original
   *             cameras, where N > 0.
   * \returns An estimated similarity transform mapping camera centers in the
   *          \c from space to camera centers in the \c to space.
   */
  virtual similarity_d
  estimate_transform(camera_map_sptr const from,
                     camera_map_sptr const to) const;

  /// Estimate the similarity transform between two corresponding landmark maps
  /**
   * Landmarks with corresponding frame IDs in the two maps are paired for
   * transform estimation. Landmarks with no corresponding frame ID in the
   * other map are ignored. An algoirithm_exception is thrown if there are no
   * shared frame IDs between the two provided maps (nothing to pair).
   *
   * \throws algorithm_exception When the from and to point sets are
   *                             misaligned, insufficient or degenerate.
   * \param from Map of original landmarks, sharing N frames with the
   *             transformed landmarks, where N > 0.
   * \param to   Map of transformed landmarks, sharing N frames with the
   *             original landmarks, where N > 0.
   * \returns An estimated similarity transform mapping landmark centers in the
   *          \c from space to camera centers in the \c to space.
   */
  virtual similarity_d
  estimate_transform(landmark_map_sptr const from,
                     landmark_map_sptr const to) const;
};


/// Shared pointer for similarity transformation algorithms
typedef boost::shared_ptr<estimate_similarity_transform> estimate_similarity_transform_sptr;


} // end algo namespace

} // end maptk namespace

#endif // MAPTK_CORE_ALGO_ESTIMATE_SIMILARITY_TRANSFORM_H
