/*ckwg +5
 * Copyright 2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include <maptk/core/algo/register.h>
#include <maptk/core/algo/track_features.h>

namespace maptk
{

namespace algo
{

/// register all algorithms in this module
void register_algorithms()
{
  simple_track_features::register_self();
}


} // end namespace algo

} // end namespace maptk
