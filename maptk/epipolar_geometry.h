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
 * \brief Header for epipolar geometry functions.
 */

#ifndef MAPTK_EPIPOLAR_GEOMETRY_H_
#define MAPTK_EPIPOLAR_GEOMETRY_H_

#include <vital/vital_config.h>
#include <maptk/maptk_export.h>

#include <vital/types/camera.h>
#include <vital/types/essential_matrix.h>
#include <vital/types/fundamental_matrix.h>
#include <vector>

namespace kwiver {
namespace maptk {


/// Test corresponding points against a fundamental matrix and mark inliers
/**
 * \param [in]  fm   the fundamental matrix
 * \param [in]  pts1 the vector or corresponding points from the first image
 * \param [in]  pts2 the vector of corresponding points from the second image
 * \param [in]  inlier_scale error distance tolerated for matches to be inliers
 * \returns     a vector of booleans, one for each point pair, the value is
 *                true if this pair is an inlier to the fundamental matrix
 */
MAPTK_EXPORT
std::vector<bool>
mark_fm_inliers(vital::fundamental_matrix const& fm,
                std::vector<vital::vector_2d> const& pts1,
                std::vector<vital::vector_2d> const& pts2,
                double inlier_scale = 1.0);


/// Compute a valid left camera from an essential matrix
/**
 * There are four valid left camera possibilities for any essential
 * matrix (assuming the right camera is the identity camera).
 * This function selects the left camera such that a corresponding
 * pair of points (in normalized coordinates) triangulates
 * in front of both cameras.
 *
 * \param [in]  e        the essential matrix
 * \param [in]  left_pt  a point in normalized coordinates of the left image
 * \param [in]  right_pt a point in normalized coordinates of the right image
 *                       that corresponds with \p left_pt
 * \returns     a camera containing the rotation and unit translation of the
 *                left camera assuming the right camera is the identity
 */
MAPTK_EXPORT
kwiver::vital::simple_camera
extract_valid_left_camera(const kwiver::vital::essential_matrix_d& e,
                          const kwiver::vital::vector_2d& left_pt,
                          const kwiver::vital::vector_2d& right_pt);


/// Compute the fundamental matrix from a pair of cameras
MAPTK_EXPORT
kwiver::vital::fundamental_matrix_sptr
fundamental_matrix_from_cameras(kwiver::vital::camera const& right_cam,
                                kwiver::vital::camera const& left_cam);


/// Compute the essential matrix from a pair of cameras
MAPTK_EXPORT
kwiver::vital::essential_matrix_sptr
essential_matrix_from_cameras(kwiver::vital::camera const& right_cam,
                              kwiver::vital::camera const& left_cam);


/// Convert an essential matrix to a fundamental matrix
MAPTK_EXPORT
kwiver::vital::fundamental_matrix_sptr
essential_matrix_to_fundamental(kwiver::vital::essential_matrix const& E,
                                kwiver::vital::camera_intrinsics const& right_cal,
                                kwiver::vital::camera_intrinsics const& left_cal);

} // end namespace maptk
} // end namespace kwiver


#endif // MAPTK_EPIPOLAR_GEOMETRY_H_
