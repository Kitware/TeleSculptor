/*ckwg +29
 * Copyright 2013-2015 by Kitware, Inc.
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
 * \brief core local_geo_cs interface
 */

#ifndef MAPTK_LOCAL_GEO_CS_H_
#define MAPTK_LOCAL_GEO_CS_H_

#include <maptk/config.h>

#include "algo/geo_map.h"
#include <vital/camera.h>
#include <vital/ins_data.h>
#include <vital/rotation.h>
#include <vital/vital_types.h>

namespace maptk
{


/// Represents a local geo coordinate system origin expressed in UTM
/**
 *  Provides functions to use global INS data to update local camera pose
 *  and local camera pose to update global INS data.
 */
class MAPTK_LIB_EXPORT local_geo_cs
{
public:
  /// Constructor
  explicit local_geo_cs(algo::geo_map_sptr alg);

  /// Set the local UTM coordinate origin
  void set_utm_origin(const kwiver::vital::vector_3d& origin) { utm_origin_ = origin; }

  /// Set the local UTM origin zone
  void set_utm_origin_zone(int zone) { utm_origin_zone_ = zone; }

  /// Access the local UTM coordinate origin
  const kwiver::vital::vector_3d& utm_origin() const { return utm_origin_; }

  /// Access the local UTM origin zone
  int utm_origin_zone() const { return utm_origin_zone_; }

  /// Access the geographic mapping algorithm
  algo::geo_map_sptr geo_map_algo() const { return geo_map_algo_; }

  /// Use the pose data provided by INS to update camera pose
  /**
   * \param ins_data    INS data packet to update the camera with
   * \param cam         The camera to be updated.
   * \param rot_offset  A rotation offset to apply to INS yaw pitch roll data
   */
  void update_camera(const kwiver::vital::ins_data& ins, kwiver::vital::camera_d& cam,
                     kwiver::vital::rotation_d const& rot_offset = kwiver::vital::rotation_d()) const;

  /// Use the camera pose to update an INS data structure
  void update_ins_data(const kwiver::vital::camera_d& cam, kwiver::vital::ins_data& ins) const;

private:
  /// An algorithm provided to compute geographic transformations
  algo::geo_map_sptr geo_map_algo_;

  /// The local coordinates origin in UTM (easting, northing, altitude)
  kwiver::vital::vector_3d utm_origin_;

  /// The UTM zone number containing the UTM origin
  int utm_origin_zone_;
};


/// Use a sequence of ins_data objects to initialize a sequence of cameras
/**
 * \param [in]     ins_map is a mapping from frame number to INS data object
 * \param [in]     base_camera is the camera to reposition at each INS pose.
 * \param [in,out] lgcs is the local geographic coordinate system used to map
 *                 lat/long to a local UTM coordinate system
 * \param [in]     Rotation offset to apply to INS yaw/pitch/roll data before
 *                 updating a camera's rotation.
 * \returns a mapping from frame number to camera
 * \note The \c lgcs object is updated only if it does not contain a valid
 *       utm_origin_zone().  If updated, the computed local origin
 *       and zone are determined from the mean camera easting and northing
 *       at zero altitude.
 */
MAPTK_LIB_EXPORT
std::map<kwiver::vital::frame_id_t, kwiver::vital::camera_sptr>
initialize_cameras_with_ins(const std::map<kwiver::vital::frame_id_t, kwiver::vital::ins_data>& ins_map,
                            const kwiver::vital::camera_d& base_camera,
                            local_geo_cs& lgcs,
                            kwiver::vital::rotation_d const& rot_offset = kwiver::vital::rotation_d());


/// Update a sequence of ins_data from a sequence of cameras and local_geo_cs
/**
 * \param [in] cam_map is a mapping from frame number to camera
 * \param [in] lgcs is the local geographic coordinate system used to map
 *             local UTM to lat/long
 * \param [in,out]  ins_map a mapping from frame_number of ins_data object to update.
 *                  If no ins_data object is found for a frame,
 *                  a new one is created
 * \note the supplied lgcs must have a valid utm_origin_zone()
 */
MAPTK_LIB_EXPORT
void
update_ins_from_cameras(const std::map<kwiver::vital::frame_id_t, kwiver::vital::camera_sptr>& cam_map,
                        const maptk::local_geo_cs& lgcs,
                        std::map<kwiver::vital::frame_id_t, kwiver::vital::ins_data>& ins_map);


} // end namespace maptk


#endif // MAPTK_LOCAL_GEO_CS_H_
