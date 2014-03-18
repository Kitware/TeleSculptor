/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief Definition for functions regarding geo reference points files
 */

#ifndef MAPTK_CORE_GEO_REFERENCE_POINTS_IO_H_
#define MAPTK_CORE_GEO_REFERENCE_POINTS_IO_H_

#include "core_config.h"
#include "landmark_map.h"
#include "local_geo_cs.h"
#include "track_set.h"
#include "types.h"


namespace maptk
{


/// Load landmarks and tracks from reference points file
/**
 * Initializes and uses a local_geo_cs object given to transform reference
 * landmarks into a local coordinate system. The newly initialized lgcs is
 * passed back up by reference. Previous initialization of the given \c lgcs
 * is overwritten.
 *
 * The reference file should be of the format
 * (lm=landmark, tNsM=track N state M):
 *
 *    lm1.x lm1.y lm1.z t1s1.frame t1s1.x t1s1.y t1s2.frame t1s2.x t1s2.y ...
 *    lm2.x lm2.y lm2.z t2s1.frame t2s1.x t2s1.y t2s2.frame t2s2.x t2s2.y ...
 *    ...
 *
 * At least 3 landmarks must be given, with at least 2 track states recorded
 * for each landmark, for transformation to converge, however more of each is
 * recommended.
 *
 * Landmark Z position, or altitude, should be given in meters.
 *
 */
MAPTK_CORE_EXPORT
void
load_reference_file(path_t const& reference_file,
                    local_geo_cs & lgcs,
                    landmark_map_sptr & ref_landmarks,
                    track_set_sptr & ref_track_set);


} // end namespace maptk

#endif // MAPTK_CORE_GEO_REFERENCE_POINTS_IO_H_
