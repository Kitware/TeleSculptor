/*ckwg +29
 * Copyright 2013-2017 by Kitware, Inc.
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


#include <maptk/maptk_export.h>

#include <vital/types/camera.h>
#include <vital/types/geo_point.h>
#include <vital/types/metadata.h>
#include <vital/types/rotation.h>
#include <vital/vital_types.h>
#include <vital/vital_config.h>

namespace kwiver {
namespace maptk {


/// Represents a local geo coordinate system origin expressed in UTM
/**
 *  Provides functions to use global INS data to update local camera pose
 *  and local camera pose to update global INS data.
 */
class MAPTK_EXPORT local_geo_cs
{
public:
  /// Constructor
  local_geo_cs();

  /// Set the geographic coordinate origin
  /**
   * Internally converts this coordinate to WGS84 UTM
   */
  void set_origin(const vital::geo_point& origin);

  /// Set the altitude of the origin (in meters)
  void set_origin_altitude(double alt) { origin_alt_ = alt; }

  /// Access the geographic coordinate of the origin
  const vital::geo_point& origin() const { return geo_origin_; }

  /// Access the geographic coordinate altituded (in meters)
  int origin_altitude() const { return origin_alt_; }

  /// Use the pose data provided by metadata to update camera pose
  /**
   * \param metadata    The metadata packet to update the camera with
   * \param cam         The camera to be updated.
   * \param rot_offset  A rotation offset to apply to metadata yaw/pitch/roll data
   */
  void update_camera(vital::metadata const& md,
                     vital::simple_camera& cam,
                     vital::rotation_d const& rot_offset = vital::rotation_d()) const;

  /// Use the camera pose to update the metadata structure
  void update_metadata(vital::simple_camera const& cam,
                       vital::metadata& md) const;

private:
  /// The local coordinates origin
  vital::geo_point geo_origin_;

  /// altitude of the local coordinate origin
  double origin_alt_;
};


/// Read a local_geo_cs from a text file
/**
 * The file format is the geographic origin in latitude (deg), longitude (deg),
 * and altitude (m) in space delimited ASCII value.  These values are read
 * into an existing local_geo_cs.
 *
 * \param [in,out] lgcs      The local geographic coordinate system that is
 *                           updated with the origin in the file.
 * \param [in]     file_path The path to the file to read.
 */
MAPTK_EXPORT
void
read_local_geo_cs_from_file(local_geo_cs& lgcs,
                            vital::path_t const& file_path);


/// Write a local_geo_cs to a text file
/**
 * The file format is the geographic origin in latitude (deg), longitude (deg),
 * and altitude (m) in space delimited ASCII value.  These values are written
 * from an existing local_geo_cs.
 *
 * \param [in] lgcs      The local geographic coordinate system to write.
 * \param [in] file_path The path to the file to write.
 */
MAPTK_EXPORT
void
write_local_geo_cs_to_file(local_geo_cs const& lgcs,
                           vital::path_t const& file_path);


/// Use a sequence of metadata objects to initialize a sequence of cameras
/**
 * \param [in]     md_map       A mapping from frame number to INS data object
 * \param [in]     base_camera  The camera to reposition at each INS pose.
 * \param [in,out] lgcs         The local geographic coordinate system used to
 *                              map lat/long to a local UTM coordinate system
 * \param [in]     rot_offset   Rotation offset to apply to yaw/pitch/roll
 *                              metadata before updating a camera's rotation.
 * \returns a mapping from frame number to camera
 * \note The \c lgcs object is updated only if it does not contain a valid
 *       utm_origin_zone().  If updated, the computed local origin
 *       and zone are determined from the mean camera easting and northing
 *       at zero altitude.
 */
MAPTK_EXPORT
std::map<vital::frame_id_t, vital::camera_sptr>
initialize_cameras_with_metadata(std::map<vital::frame_id_t,
                                          vital::metadata_sptr> const& md_map,
                                 vital::simple_camera const& base_camera,
                                 local_geo_cs& lgcs,
                                 vital::rotation_d const& rot_offset = vital::rotation_d());


/// Update a sequence of metadata from a sequence of cameras and local_geo_cs
/**
 * \param [in]      cam_map   A mapping from frame number to camera
 * \param [in]      lgcs      The local geographic coordinate system used to
 *                            map local UTM to lat/long
 * \param [in,out]  md_map    A mapping from frame_number of metadata objects
 *                            to update.  If no metadata object is found for
 *                            a frame, a new one is created.
 * \note the supplied lgcs must have a valid utm_origin_zone()
 */
MAPTK_EXPORT
void
update_metadata_from_cameras(std::map<vital::frame_id_t, vital::camera_sptr> const& cam_map,
                             maptk::local_geo_cs const& lgcs,
                             std::map<vital::frame_id_t, vital::metadata_sptr>& md_map);


} // end namespace maptk
} // end namespace kwiver


#endif // MAPTK_LOCAL_GEO_CS_H_
