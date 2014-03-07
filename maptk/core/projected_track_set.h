/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief Header file for functions relating to generating projected
 * tracks from a sequence of landmarks and camera parameters.
 */

#ifndef MAPTK_PROJECTED_TRACK_SET_H_
#define MAPTK_PROJECTED_TRACK_SET_H_

#include "core_config.h"

#include <boost/shared_ptr.hpp>

#include "track_set.h"
#include "camera_map.h"
#include "landmark_map.h"

namespace maptk
{


/// Create tracks by projecting the landmarks into the cameras
track_set_sptr
MAPTK_CORE_EXPORT
projected_tracks(landmark_map_sptr landmarks, camera_map_sptr cameras);


} // end namespace maptk


#endif // MAPTK_PROJECTED_TRACK_SET_H_
