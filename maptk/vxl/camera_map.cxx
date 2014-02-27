/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief Implementation of map from frame IDs to vpgl cameras
 */

#include <maptk/vxl/camera_map.h>
#include <maptk/vxl/camera.h>
#include <boost/foreach.hpp>

namespace maptk
{

namespace vxl
{


/// Return a map from integer IDs to camera shared pointers
maptk::camera_map::map_camera_t
camera_map::cameras() const
{
  map_camera_t maptk_cameras;

  BOOST_FOREACH(const map_vcam_t::value_type& c, data_)
  {
    camera_sptr cam = vpgl_camera_to_maptk(c.second);
    maptk_cameras.insert(std::make_pair(c.first, cam));
  }

  return maptk_cameras;
}


/// Convert any camera map to a vpgl camera map
camera_map::map_vcam_t
camera_map_to_vpgl(const maptk::camera_map& cam_map)
{
  // if the camera map already contains a vpgl representation
  // then return the existing vpgl data
  if( const vxl::camera_map* m =
          dynamic_cast<const vxl::camera_map*>(&cam_map) )
  {
    return m->vpgl_cameras();
  }
  camera_map::map_vcam_t vmap;
  BOOST_FOREACH(const camera_map::map_camera_t::value_type& c,
                cam_map.cameras())
  {
    vpgl_perspective_camera<double> vcam;
    if( const camera_d* mcamd = dynamic_cast<const camera_d*>(c.second.get()) )
    {
      maptk_to_vpgl_camera(*mcamd, vcam);
    }
    else if( const camera_f* mcamf = dynamic_cast<const camera_f*>(c.second.get()) )
    {
      maptk_to_vpgl_camera(camera_d(*mcamf), vcam);
    }
    else
    {
      //TODO should throw an exception here
    }
    vmap.insert(std::make_pair(c.first, vcam));
  }
  return vmap;
}


} // end namespace vxl

} // end namespace maptk
