/*ckwg +5
 * Copyright 2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include "local_geo_cs.h"


namespace maptk
{


/// Constructor
local_geo_cs
::local_geo_cs(algo::geo_map_sptr alg)
: geo_map_algo_(alg),
  utm_origin_(0.0, 0.0, 0.0),
  utm_origin_zone_(-1)
{
}


/// Use the pose data provided by INS to update camera pose
void
local_geo_cs
::update_camera(const ins_data& ins, camera_d& cam)
{
  if( !geo_map_algo_ )
  {
    return;
  }
  cam.set_rotation(rotation_d(ins.yaw, ins.pitch, ins.roll));
  double x,y;
  int zone;
  bool is_north_hemi;
  geo_map_algo_->latlon_to_utm(ins.lat, ins.lon,
                               x, y, zone, is_north_hemi, utm_origin_zone_);
  cam.set_center(vector_3d(x, y, ins.alt) - utm_origin_);
}


/// Use the camera pose to update an INS data structure
void
local_geo_cs
::update_ins_data(const camera_d& cam, ins_data& ins)
{
  if( !geo_map_algo_ )
  {
    return;
  }
  cam.rotation().get_yaw_pitch_roll(ins.yaw, ins.pitch, ins.roll);
  vector_3d c = cam.center() + utm_origin_;
  int zone;
  bool is_north_hemi;
  geo_map_algo_->utm_to_latlon(c.x(), c.y(), zone, is_north_hemi,
                               ins.lat, ins.lon);
  ins.alt = c.z();
}


} // end namespace maptk
