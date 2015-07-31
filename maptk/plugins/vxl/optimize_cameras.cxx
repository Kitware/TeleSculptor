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
* \brief Header defining VXL algorithm implementation of camera optimization.
*/

#include "optimize_cameras.h"

#include <map>
#include <vector>
#include <utility>

#include <boost/foreach.hpp>

#include <vital/exceptions.h>

#include <maptk/plugins/vxl/camera.h>
#include <maptk/plugins/vxl/camera_map.h>

#include <vcl_vector.h>
#include <vgl/vgl_homg_point_3d.h>
#include <vgl/vgl_point_2d.h>
#include <vnl/algo/vnl_levenberg_marquardt.h>
#include <vnl/vnl_double_3.h>
#include <vpgl/algo/vpgl_optimize_camera.h>

using namespace kwiver::vital;

namespace kwiver {
namespace maptk {

namespace vxl
{

namespace // anonymous
{

/// Reproduction of vpgl_optimize_camera::opt_orient_pos(...), but without
/// trace statement.
vpgl_perspective_camera<double>
maptk_opt_orient_pos(vpgl_perspective_camera<double> const& camera,
                     vcl_vector<vgl_homg_point_3d<double> > const& world_points,
                     vcl_vector<vgl_point_2d<double> > const& image_points)
{
  const vpgl_calibration_matrix<double>& K = camera.get_calibration();
  vgl_point_3d<double> c = camera.get_camera_center();
  const vgl_rotation_3d<double>& R = camera.get_rotation();

  // compute the Rodrigues vector from the rotation
  vnl_double_3 w = R.as_rodrigues();

  vpgl_orientation_position_lsqr lsqr_func(K,world_points,image_points);
  vnl_levenberg_marquardt lm(lsqr_func);
  vnl_vector<double> params(6);
  params[0]=w[0];   params[1]=w[1];   params[2]=w[2];
  params[3]=c.x();  params[4]=c.y();  params[5]=c.z();
  lm.minimize(params);
  vnl_double_3 w_min(params[0],params[1],params[2]);
  vgl_homg_point_3d<double> c_min(params[3], params[4], params[5]);

  return vpgl_perspective_camera<double>(K, c_min, vgl_rotation_3d<double>(w_min) );
}

}


void
optimize_cameras
::optimize(kwiver::vital::camera_map_sptr & cameras,
           kwiver::vital::track_set_sptr tracks,
            kwiver::vital::landmark_map_sptr landmarks) const
{
  if (!cameras || !tracks || !landmarks)
  {
    throw kwiver::vital::invalid_value("One or more input data pieces are Null!");
  }
  typedef kwiver::vital::camera_map::map_camera_t map_camera_t;
  typedef maptk::vxl::camera_map::map_vcam_t map_vcam_t;
  typedef kwiver::vital::landmark_map::map_landmark_t map_landmark_t;

  // extract data from containers
  map_vcam_t vcams = camera_map_to_vpgl(*cameras);
  map_landmark_t lms = landmarks->landmarks();
  std::vector<track_sptr> trks = tracks->tracks();

  // Compose a map of frame IDs to a nested map of track ID to the state on
  // that frame number. Using a pointer with the track state to ease the burden
  // on memory.
  typedef std::map< kwiver::vital::track_id_t, kwiver::vital::track::track_state const* > inner_map_t;
  typedef std::map< kwiver::vital::frame_id_t, inner_map_t > states_map_t;

  states_map_t states_map;
  // O( len(trks) * avg_t_len )
  BOOST_FOREACH(kwiver::vital::track_sptr const& t, trks)
  {
    // Only record a state if there is a corresponding landmark for the
    // track (constant-time check), the track state has a feature and thus a
    // location (constant-time check), and if we have a camera on the track
    // state's frame (constant-time check).
    if (lms.count(t->id()))
    {
      for (track::history_const_itr tsi = t->begin();
           tsi != t->end();
           ++tsi)
      {
        if (tsi->feat && vcams.count(tsi->frame_id))
        {
          states_map[tsi->frame_id][t->id()] = &(*tsi);
        }
      }
    }
  }

  // For each camera in the input map, create corresponding point sets for 2D
  // and 3D coordinates of tracks and matching landmarks, respectively, for
  // that camera's frame.
  map_camera_t optimized_cameras;
  vcl_vector< vgl_point_2d<double> > pts_2d;
  vcl_vector< vgl_homg_point_3d<double> > pts_3d;
  vector_2d tmp_2d;
  vector_3d tmp_3d;
  BOOST_FOREACH(map_vcam_t::value_type const& p, vcams)
  {
    pts_2d.clear();
    pts_3d.clear();

    // Construct 2d<->3d correspondences
    BOOST_FOREACH(inner_map_t::value_type const& q, states_map[p.first])
    {
      // Already guaranteed that feat and landmark exists above.
      tmp_2d = q.second->feat->loc();
      tmp_3d = lms[q.first]->loc();
      pts_2d.push_back(vgl_point_2d<double>(tmp_2d.x(), tmp_2d.y()));
      pts_3d.push_back(vgl_homg_point_3d<double>(tmp_3d.x(), tmp_3d.y(), tmp_3d.z()));
    }

    //optimized_cameras[p.first] =
    //  vpgl_camera_to_maptk(vpgl_optimize_camera::opt_orient_pos(p.second,
    //                                                            pts_3d,
    //                                                            pts_2d));
    optimized_cameras[p.first] =
      vpgl_camera_to_maptk(maptk_opt_orient_pos(p.second, pts_3d, pts_2d));
  }

  cameras = camera_map_sptr(new simple_camera_map(optimized_cameras));
}


} // end namespace vxl

} // end namespace maptk
} // end namespace kwiver
