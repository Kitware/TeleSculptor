/*ckwg +29
 * Copyright 2015 by Kitware, Inc.
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

#include <maptk/config.h>

#include <vital/types/track_set.h>
#include <Eigen/Sparse>


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
MAPTK_LIB_EXPORT
Eigen::SparseMatrix<unsigned int>
match_matrix(const vital::track_set_sptr tracks,
                   std::vector<vital::frame_id_t>& frames);

} // end namespace maptk
} // end namespace kwiver


#endif // MAPTK_MATCH_MATRIX_H_
