/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include <maptk/core/algo/register.h>
#include <maptk/core/algo/track_features_default.h>
#include <maptk/core/algo/match_features_homography.h>

namespace maptk
{

namespace algo
{

/// register all algorithms in this module
void register_algorithms()
{
  track_features_default::register_self();
  match_features_homography::register_self();
}


} // end namespace algo

} // end namespace maptk
