/*ckwg +5
 * Copyright 2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_LOCAL_GEO_CS_H_
#define MAPTK_LOCAL_GEO_CS_H_

#include "ins_data.h"
#include "camera.h"
#include "algo/geo_map.h"


namespace maptk
{


/// Represents a local geo coordinate system origin expressed in UTM
/**
 *  Provides functions to use global INS data to update local camera pose
 *  and local camera pose to update global INS data.
 */
class local_geo_cs
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
  void update_camera(const ins_data& ins, camera_d& cam);

  /// Use the camera pose to update an INS data structure
  void update_ins_data(const camera_d& cam, ins_data& ins);

private:
  /// An algorithm provided to compute geographic transformations
  algo::geo_map_sptr geo_map_algo_;

  /// The local coordinates origin in UTM (easting, northing, altitude)
  vector_3d utm_origin_;

  /// The UTM zone number containing the UTM origin
  int utm_origin_zone_;
};


} // end namespace maptk


#endif // MAPTK_LOCAL_GEO_CS_H_
