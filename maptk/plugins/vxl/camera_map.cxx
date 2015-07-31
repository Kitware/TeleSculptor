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
 * \brief Implementation of map from frame IDs to vpgl cameras
 */

#include "camera_map.h"

#include <boost/foreach.hpp>

#include <maptk/plugins/vxl/camera.h>

using namespace kwiver::vital;

namespace kwiver {
namespace maptk {

namespace vxl
{


/// Return a map from integer IDs to camera shared pointers
vital::camera_map::map_camera_t
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
camera_map_to_vpgl(const vital::camera_map& cam_map)
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
} // end namespace kwiver
