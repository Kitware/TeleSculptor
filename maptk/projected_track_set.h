/*ckwg +29
 * Copyright 2014-2015 by Kitware, Inc.
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
 * \brief Header file for functions relating to generating projected
 * tracks from a sequence of landmarks and camera parameters.
 */

#ifndef MAPTK_PROJECTED_TRACK_SET_H_
#define MAPTK_PROJECTED_TRACK_SET_H_

#include <boost/shared_ptr.hpp>

#include <maptk/config.h>

#include <vital/types/track_set.h>
#include <vital/types/camera_map.h>
#include <vital/types/landmark_map.h>

namespace kwiver {
namespace maptk {


/// Use the cameras to project the landmarks back into their images.
/**
 * \param landmarks input landmark locations
 * \param cameras input camera map
 * \return track set generated via the projection
 */
kwiver::vital::track_set_sptr
MAPTK_LIB_EXPORT
projected_tracks(kwiver::vital::landmark_map_sptr landmarks,
                 kwiver::vital::camera_map_sptr cameras);


} // end namespace maptk
} // end namespace kwiver


#endif // MAPTK_PROJECTED_TRACK_SET_H_
