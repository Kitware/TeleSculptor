/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include "local_geo_cs.h"
#include <boost/math/constants/constants.hpp>


namespace maptk
{


/// scale factor converting radians to degrees
const double rad2deg = 180.0 / boost::math::constants::pi<double>();
/// scale factor converting degrees to radians
const double deg2rad = boost::math::constants::pi<double>() / 180.0;


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
  cam.set_rotation(rotation_d(ins.yaw * deg2rad,
                              ins.pitch * deg2rad,
                              ins.roll * deg2rad));
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
  ins.yaw *= rad2deg;
  ins.pitch *= rad2deg;
  ins.roll *= rad2deg;
  vector_3d c = cam.get_center() + utm_origin_;
  geo_map_algo_->utm_to_latlon(c.x(), c.y(), utm_origin_zone_, true,
                               ins.lat, ins.lon);
  ins.alt = c.z();
}


} // end namespace maptk
