/*ckwg +5
* Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
* KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
* Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
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

#include <maptk/core/exceptions.h>
#include <maptk/vxl/camera.h>
#include <maptk/vxl/camera_map.h>

#include <vcl_vector.h>
#include <vgl/vgl_homg_point_3d.h>
#include <vgl/vgl_point_2d.h>
#include <vpgl/algo/vpgl_optimize_camera.h>


namespace maptk
{

namespace vxl
{


void
optimize_cameras
::optimize(camera_map_sptr & cameras,
           track_set_sptr tracks,
           landmark_map_sptr landmarks) const
{
  if (!cameras || !tracks || !landmarks)
  {
    throw invalid_value("One or more input data pieces are Null!");
  }
  typedef maptk::camera_map::map_camera_t map_camera_t;
  typedef maptk::vxl::camera_map::map_vcam_t map_vcam_t;
  typedef maptk::landmark_map::map_landmark_t map_landmark_t;

  // extract data from containers
  map_vcam_t vcams = camera_map_to_vpgl(*cameras);
  map_landmark_t lms = landmarks->landmarks();
  std::vector<track_sptr> trks = tracks->tracks();

  // Compose a map of frame IDs to a nested map of track ID to the state on
  // that frame number. Using a pointer with the track state to ease the burden
  // on memory.
  typedef std::map< track_id_t, track::track_state const* > inner_map_t;
  typedef std::map< frame_id_t, inner_map_t > states_map_t;

  states_map_t states_map;
  // O( len(trks) * avg_t_len )
  BOOST_FOREACH(track_sptr const& t, trks)
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

    optimized_cameras[p.first] =
      vpgl_camera_to_maptk(vpgl_optimize_camera::opt_orient_pos(p.second,
                                                                pts_3d,
                                                                pts_2d));
  }

  cameras = camera_map_sptr(new simple_camera_map(optimized_cameras));
}


}

}
