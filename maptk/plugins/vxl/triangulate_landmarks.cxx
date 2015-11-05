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
 * \brief Implementation of VXL triangulate landmarks algorithm
 */

#include "triangulate_landmarks.h"

#include <set>

#include <vital/vital_foreach.h>

#include <maptk/plugins/vxl/camera_map.h>

#include <vpgl/algo/vpgl_triangulate_points.h>

using namespace kwiver::vital;

namespace kwiver {
namespace maptk {

namespace vxl
{

/// Private implementation class
class triangulate_landmarks::priv
{
public:
  /// Constructor
  priv()
  {
  }

  priv(const priv& other)
  {
  }

  /// parameters - none yet
};


/// Constructor
triangulate_landmarks
::triangulate_landmarks()
: d_(new priv)
{
}


/// Copy Constructor
triangulate_landmarks
::triangulate_landmarks(const triangulate_landmarks& other)
: d_(new priv(*other.d_))
{
}


/// Destructor
triangulate_landmarks
::~triangulate_landmarks()
{
}


/// Get this algorithm's \link vital::config_block configuration block \endlink
vital::config_block_sptr
triangulate_landmarks
::get_configuration() const
{
  // get base config from base class
  vital::config_block_sptr config = vital::algo::triangulate_landmarks::get_configuration();
  return config;
}


/// Set this algorithm's properties via a config block
void
triangulate_landmarks
::set_configuration(vital::config_block_sptr in_config)
{
}


/// Check that the algorithm's currently configuration is valid
bool
triangulate_landmarks
::check_configuration(vital::config_block_sptr config) const
{
  return true;
}


/// Triangulate the landmark locations given sets of cameras and tracks
void
triangulate_landmarks
::triangulate(vital::camera_map_sptr cameras,
              vital::track_set_sptr tracks,
              vital::landmark_map_sptr& landmarks) const
{
  if( !cameras || !landmarks || !tracks )
  {
    // TODO throw an exception for missing input data
    return;
  }
  typedef vxl::camera_map::map_vcam_t map_vcam_t;
  typedef vital::landmark_map::map_landmark_t map_landmark_t;

  // extract data from containers
  map_vcam_t vcams = camera_map_to_vpgl(*cameras);
  map_landmark_t lms = landmarks->landmarks();
  std::vector<track_sptr> trks = tracks->tracks();

  // build a track map by id
  typedef std::map<track_id_t, track_sptr> track_map_t;
  track_map_t track_map;
  VITAL_FOREACH(const track_sptr& t, trks)
  {
    track_map[t->id()] = t;
  }

  // the set of landmark ids which failed to triangulation
  std::set<landmark_id_t> failed_landmarks;

  map_landmark_t triangulated_lms;
  VITAL_FOREACH(const map_landmark_t::value_type& p, lms)
  {
    // get the corresponding track
    track_map_t::const_iterator t_itr = track_map.find(p.first);
    if (t_itr == track_map.end())
    {
      // there is no track for the provided landmark
      continue;
    }
    const track& t = *t_itr->second;

    // extract the cameras and image points for this landmarks
    std::vector<vpgl_perspective_camera<double> > lm_cams;
    std::vector<vgl_point_2d<double> > lm_image_pts;

    for (track::history_const_itr tsi = t.begin(); tsi != t.end(); ++tsi)
    {
      if (!tsi->feat)
      {
        // there is no valid feature for this track state
        continue;
      }
      map_vcam_t::const_iterator c_itr = vcams.find(tsi->frame_id);
      if (c_itr == vcams.end())
      {
        // there is no camera for this track state.
        continue;
      }
      lm_cams.push_back(c_itr->second);
      vital::vector_2d pt = tsi->feat->loc();
      lm_image_pts.push_back(vgl_point_2d<double>(pt.x(), pt.y()));
    }

    // if we found at least two views of this landmark, triangulate
    if (lm_cams.size() > 1)
    {
      vital::vector_3d lm_loc = p.second->loc();
      vgl_point_3d<double> pt3d(lm_loc.x(), lm_loc.y(), lm_loc.z());
      double error = vpgl_triangulate_points::triangulate(lm_image_pts,
                                                          lm_cams, pt3d);
      bool bad_triangulation = false;
      vgl_homg_point_3d<double> hpt3d(pt3d);
      VITAL_FOREACH(vpgl_perspective_camera<double> const& cam, lm_cams)
      {
        if(cam.is_behind_camera(hpt3d))
        {
          bad_triangulation = true;
          failed_landmarks.insert(p.first);
          break;
        }
      }
      if( !bad_triangulation )
      {
        landmark_d* lm = new landmark_d(vital::vector_3d(pt3d.x(), pt3d.y(), pt3d.z()));
        lm->set_covar(covariance_3d(error));
        triangulated_lms[p.first] = landmark_sptr(lm);
      }
    }
  }
  if( !failed_landmarks.empty() )
  {
    std::cerr << "failed to triangulate " << failed_landmarks.size()
              << " of " << lms.size() << " landmarks" << std::endl;
  }
  landmarks = landmark_map_sptr(new simple_landmark_map(triangulated_lms));
}


} // end namespace vxl

} // end namespace maptk
} // end namespace kwiver
