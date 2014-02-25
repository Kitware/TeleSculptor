/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief core local_geo_cs interface
 */

#ifndef MAPTK_LOCAL_GEO_CS_H_
#define MAPTK_LOCAL_GEO_CS_H_

#include "core_config.h"
#include "types.h"

#include "algo/geo_map.h"
#include "camera.h"
#include "ins_data.h"

namespace maptk
{


/// Represents a local geo coordinate system origin expressed in UTM
/**
 *  Provides functions to use global INS data to update local camera pose
 *  and local camera pose to update global INS data.
 */
class MAPTK_CORE_EXPORT local_geo_cs
{
public:
  /// Constructor
  explicit local_geo_cs(algo::geo_map_sptr alg);

  /// Set the local UTM coordinate origin
  void set_utm_origin(const vector_3d& origin) { utm_origin_ = origin; }

  /// Set the local UTM origin zone
  void set_utm_origin_zone(int zone) { utm_origin_zone_ = zone; }

  /// Access the local UTM coordinate origin
  const vector_3d& utm_origin() const { return utm_origin_; }

  /// Access the local UTM origin zone
  int utm_origin_zone() const { return utm_origin_zone_; }

  /// Access the geographic mapping algorithm
  algo::geo_map_sptr geo_map_algo() const { return geo_map_algo_; }

  /// Use the pose data provided by INS to update camera pose
  void update_camera(const ins_data& ins, camera_d& cam) const;

  /// Use the camera pose to update an INS data structure
  void update_ins_data(const camera_d& cam, ins_data& ins) const;

private:
  /// An algorithm provided to compute geographic transformations
  algo::geo_map_sptr geo_map_algo_;

  /// The local coordinates origin in UTM (easting, northing, altitude)
  vector_3d utm_origin_;

  /// The UTM zone number containing the UTM origin
  int utm_origin_zone_;
};


/// Use a sequence of ins_data objects to initialize a sequence of cameras
/**
 * \param [in]     ins_map is a mapping from frame number to INS data object
 * \param [in]     base_camera is the camera to reposition at each INS pose.
 * \param [in,out] lgcs is the local geographic coordinate system used to map
 *                 lat/long to a local UTM coordinate system
 * \returns a mapping from frame number to camera
 * \note The \c lgcs object is updated only if it does not contain a valid
 *       utm_origin_zone().  If updated, the computed local origin
 *       and zone are determined from the mean camera easting and northing
 *       at zero altitude.
 */
MAPTK_CORE_EXPORT
std::map<frame_id_t, camera_sptr>
initialize_cameras_with_ins(const std::map<frame_id_t, ins_data>& ins_map,
                            const camera_d& base_camera,
                            local_geo_cs& lgcs);


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
MAPTK_CORE_EXPORT
void
update_ins_from_cameras(const std::map<frame_id_t, camera_sptr>& cam_map,
                        const local_geo_cs& lgcs,
                        std::map<frame_id_t, ins_data>& ins_map);


} // end namespace maptk


#endif // MAPTK_LOCAL_GEO_CS_H_
