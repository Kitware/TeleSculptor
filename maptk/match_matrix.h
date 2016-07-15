/*ckwg +29
 * Copyright 2015-2016 by Kitware, Inc.
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
 * \brief Header for match matrix computation
 */

#ifndef MAPTK_MATCH_MATRIX_H_
#define MAPTK_MATCH_MATRIX_H_


#include <vital/vital_config.h>
#include <maptk/maptk_export.h>

#include <vital/types/track_set.h>
#include <Eigen/Sparse>

#include <map>


namespace kwiver {
namespace maptk {


/// Compute the match matrix from a track set
/**
 *  This function computes an NxN integer symmetric matrix such that matrix
 *  element (i,j) is the number of feature tracks with corresponding points
 *  on both frames i and j.  The diagonal (i,i) is the number of features
 *  on frame i.  The frame ids corresponding to each row/column are returned
 *  in a vector.
 *
 *  \param[in]     tracks  The tracks from which to extract the match matrix
 *  \param[in,out] frames  The vector of frame ids used in the match matrix.
 *                         If empty, this will be filled in will all available
 *                         frame ids in the track set.
 *  \return an NxN symmetric match matrix
 */
MAPTK_EXPORT
Eigen::SparseMatrix<unsigned int>
match_matrix(vital::track_set_sptr tracks,
             std::vector<vital::frame_id_t>& frames);


/// Compute a score for each track based on its importance to the match matrix.
/**
 * Using the match matrix (as computed by vital::match_matrix) assign a score
 * to each track that is proportional to that tracks importance in reproducing
 * the matrix.  That is, the top N scoring tracks should provide the best
 * approximation to the coverage of match matrix if only those N tracks are
 * used.  Tracks are scored as the sum of one over the mm(i,j) where mm(i,j)
 * is the match matrix entry at (i,j) for every frame i and j in the track.
 *
 */
MAPTK_EXPORT
std::map<vital::track_id_t, double>
match_matrix_track_importance(vital::track_set_sptr tracks,
                              std::vector<vital::frame_id_t> const& frames,
                              Eigen::SparseMatrix<unsigned int> const& mm);
} // end namespace maptk
} // end namespace kwiver


#endif // MAPTK_MATCH_MATRIX_H_
