/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief core algorithms registration implementation
 */

#include <maptk/core/algo/register.h>

#include <maptk/core/algo/close_loops_bad_frames_only.h>
#include <maptk/core/algo/compute_ref_homography_default.h>
#include <maptk/core/algo/convert_image.h>
#include <maptk/core/algo/hierarchical_bundle_adjust.h>
#include <maptk/core/algo/match_features_homography.h>
#include <maptk/core/algo/track_features_default.h>

namespace maptk
{

namespace algo
{

/// register all algorithms in this module
void register_algorithms()
{
  close_loops_bad_frames_only::register_self();
  compute_ref_homography_default::register_self();
  convert_image_default::register_self();
  hierarchical_bundle_adjust::register_self();
  match_features_homography::register_self();
  track_features_default::register_self();
}


} // end namespace algo

} // end namespace maptk
