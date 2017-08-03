/*ckwg +29
 * Copyright 2014-2017 by Kitware, Inc.
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
 * \brief Definition for functions regarding geo reference points files
 */

#ifndef MAPTK_GEO_REFERENCE_POINTS_IO_H_
#define MAPTK_GEO_REFERENCE_POINTS_IO_H_


#include <vital/vital_config.h>
#include <maptk/maptk_export.h>

#include "local_geo_cs.h"

#include <vital/types/landmark_map.h>
#include <vital/types/feature_track_set.h>
#include <vital/vital_types.h>


namespace kwiver {
namespace maptk {


/// Load landmarks and feature tracks from reference points file
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
MAPTK_EXPORT
void
load_reference_file(vital::path_t const& reference_file,
                    local_geo_cs & lgcs,
                    vital::landmark_map_sptr & ref_landmarks,
                    vital::feature_track_set_sptr & ref_track_set);


} // end namespace maptk
} // end namespace kwiver

#endif // MAPTK_GEO_REFERENCE_POINTS_IO_H_
