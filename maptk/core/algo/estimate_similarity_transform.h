/*ckwg +29
 * Copyright 2013-2014 by Kitware, Inc.
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
  virtual std::string type_name() const { return "estimate_similarity_transform"; }

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
