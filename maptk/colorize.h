/*ckwg +29
 * Copyright 2016-2017 by Kitware, Inc.
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
 * \brief Header for maptk::colorize functions
 */

#ifndef MAPTK_COLORIZE_H_
#define MAPTK_COLORIZE_H_

#include <maptk/maptk_export.h>

#include <vital/types/feature_set.h>
#include <vital/types/image_container.h>
#include <vital/types/landmark_map.h>
#include <vital/types/feature_track_set.h>


namespace kwiver {
namespace maptk {

/// Extract feature colors from a frame image
/*
 * This function extracts the feature colors from a supplied frame image and
 * applies them to all features in a feature set by sampling the image at each
 * feature's location.
 *
 *  \param [in] features a set of features for which to assign colors
 *  \param [in] image the image from which to take colors
 *  \return a feature set with updated features
 */
MAPTK_EXPORT
vital::feature_set_sptr extract_feature_colors(
  vital::feature_set const& features,
  vital::image_container const& image);

/// Extract feature colors from a frame image
/**
 * This function extracts the feature colors from a supplied frame image and
 * applies them to all features in the input track set with the same frame
 * number.
 *
 *  \param [in] tracks a set of feature tracks in which to colorize feature points
 *  \param [in] image the image from which to take colors
 *  \param [in] frame_id the frame number of the image
 *  \return a track set with updated features
 */
MAPTK_EXPORT
vital::feature_track_set_sptr extract_feature_colors(
  vital::feature_track_set const& tracks,
  vital::image_container const& image,
  vital::frame_id_t frame_id);

/// Compute colors for landmarks
/**
 * This function computes landmark colors by taking the average color of all
 * associated feature points.
 *
 *  \param [in] landmarks a set of landmarks to be colored
 *  \param [in] tracks feature tracks to be used for computing landmark colors
 *  \return a set of colored landmarks
 */
MAPTK_EXPORT
vital::landmark_map_sptr compute_landmark_colors(
  vital::landmark_map const& landmarks,
  vital::feature_track_set const& tracks);

} // end namespace maptk
} // end namespace kwiver


#endif
