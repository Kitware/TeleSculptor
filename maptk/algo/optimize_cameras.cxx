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
 * \brief Instantiation of \link maptk::algo::algorithm_def algorithm_def<T>
 *        \endlink for \link maptk::algo::optimize_cameras
 *        optimize_cameras \endlink
 */

#include <maptk/algo/algorithm.txx>
#include <maptk/algo/optimize_cameras.h>


/// \cond DoxygenSuppress
INSTANTIATE_ALGORITHM_DEF(maptk::algo::optimize_cameras);
/// \endcond

namespace maptk
{

namespace algo
{


/// Optimize camera parameters given sets of landmarks and tracks
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
  typedef maptk::landmark_map::map_landmark_t map_landmark_t;

  // extract data from containers
  map_camera_t cams = cameras->cameras();
  map_landmark_t lms = landmarks->landmarks();
  std::vector<track_sptr> trks = tracks->tracks();

  // Compose a map of frame IDs to a nested map of track ID to the state on
  // that frame number.
  typedef std::map< track_id_t, feature_sptr > inner_map_t;
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
        if (tsi->feat && cams.count(tsi->frame_id))
        {
          states_map[tsi->frame_id][t->id()] = tsi->feat;
        }
      }
    }
  }

  // For each camera in the input map, create corresponding point sets for 2D
  // and 3D coordinates of tracks and matching landmarks, respectively, for
  // that camera's frame.
  map_camera_t optimized_cameras;
  std::vector< feature_sptr > v_feat;
  std::vector< landmark_sptr > v_lms;
  BOOST_FOREACH(map_camera_t::value_type const& p, cams)
  {
    v_feat.clear();
    v_lms.clear();

    // Construct 2d<->3d correspondences
    BOOST_FOREACH(inner_map_t::value_type const& q, states_map[p.first])
    {
      // Already guaranteed that feat and landmark exists above.
      v_feat.push_back(q.second);
      v_lms.push_back(lms[q.first]);
    }

    camera_sptr cam = p.second;
    this->optimize(cam, v_feat, v_lms);
    optimized_cameras[p.first] = cam;
  }

  cameras = camera_map_sptr(new simple_camera_map(optimized_cameras));
}



} // end namespace algo
} // end namespace maptk
