/*ckwg +29
 * Copyright 2014 by Kitware, Inc.
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
 * \brief Implementation of VXL camera and landmark initialization algorithm
 */

#include <maptk/core/algo/estimate_essential_matrix.h>
#include <maptk/core/exceptions.h>
#include <maptk/vxl/initialize_cameras_landmarks.h>
#include <maptk/vxl/camera_map.h>
#include <maptk/vxl/camera.h>
#include <boost/foreach.hpp>
#include <deque>

#include <vpgl/vpgl_essential_matrix.h>


namespace maptk
{

namespace vxl
{


/// Private implementation class
class initialize_cameras_landmarks::priv
{
public:
  /// Constructor
  priv()
  : verbose(false),
    base_camera()
  {
  }

  priv(const priv& other)
  : verbose(other.verbose),
    base_camera(other.base_camera)
  {
  }

  /// Construct and initialized camera for \a frame
  camera_sptr init_camera(frame_id_t frame, frame_id_t last_frame,
                          const camera_map::map_camera_t& cams,
                          const std::vector<track_sptr>& trks,
                          const landmark_map::map_landmark_t& lms) const;

  bool verbose;
  camera_d base_camera;
  algo::estimate_essential_matrix_sptr e_estimator;
};


/// Construct and initialized camera for \a frame
camera_sptr
initialize_cameras_landmarks::priv
::init_camera(frame_id_t frame, frame_id_t last_frame,
              const camera_map::map_camera_t& cams,
              const std::vector<track_sptr>& trks,
              const landmark_map::map_landmark_t& lms) const
{
  // extract coresponding image points
  std::vector<vector_2d> pts_right, pts_left;
  for(unsigned int i=0; i<trks.size(); ++i)
  {
    pts_right.push_back(trks[i]->find(last_frame)->feat->loc());
    pts_left.push_back(trks[i]->find(frame)->feat->loc());
  }

  // compute the essential matrix from the corresponding points
  camera_map::map_camera_t::const_iterator ci = cams.find(last_frame);
  if( ci == cams.end() )
  {
    throw invalid_value("Camera for last frame not provided.");
  }
  camera_sptr prev_cam = ci->second;
  camera_intrinsics_d cal_right = prev_cam->intrinsics();
  const camera_intrinsics_d& cal_left = base_camera.get_intrinsics();
  std::vector<bool> inliers;
  matrix_3x3d E = e_estimator->estimate(pts_right, pts_left,
                                        cal_right, cal_left,
                                        inliers, 2.0);

  unsigned num_inliers = static_cast<unsigned>(std::count(inliers.begin(),
                                                          inliers.end(), true));
  std::cout << "E matrix num inliers = "<<num_inliers<<std::endl;

  // get the first inlier index
  unsigned int inlier_idx = 0;
  for(; inlier_idx < inliers.size() && !inliers[inlier_idx]; ++inlier_idx);

  // compute the corresponding camera rotation and translation (up to scale)
  vpgl_essential_matrix<double> vE(vnl_double_3x3(E.data()));
  vpgl_perspective_camera<double> vcam;
  vector_2d left_pt = cal_left.unmap(pts_left[inlier_idx]);
  vector_2d right_pt = cal_right.unmap(pts_right[inlier_idx]);
  vgl_point_2d<double> vleft_pt(left_pt.x(), left_pt.y());
  vgl_point_2d<double> vright_pt(right_pt.x(), right_pt.y());
  extract_left_camera(vE, vleft_pt, vright_pt, vcam);
  camera_d cam;
  vpgl_camera_to_maptk(vcam, cam);
  cam.set_intrinsics(base_camera.get_intrinsics());

  // adjust pose relative to the previous camera
  vector_3d new_t = cam.get_rotation() * prev_cam->translation()
                  + cam.translation();
  cam.set_rotation(cam.get_rotation() * prev_cam->rotation());
  cam.set_translation(new_t);

  return camera_sptr(new camera_d(cam));
}


/// Constructor
initialize_cameras_landmarks
::initialize_cameras_landmarks()
: d_(new priv)
{
}


/// Copy Constructor
initialize_cameras_landmarks
::initialize_cameras_landmarks(const initialize_cameras_landmarks& other)
: d_(new priv(*other.d_))
{
}


/// Destructor
initialize_cameras_landmarks
::~initialize_cameras_landmarks()
{
}


/// Get this algorithm's \link maptk::config_block configuration block \endlink
config_block_sptr
initialize_cameras_landmarks
::get_configuration() const
{
  // get base config from base class
  config_block_sptr config =
      maptk::algo::initialize_cameras_landmarks::get_configuration();

  const camera_intrinsics_d& K = d_->base_camera.get_intrinsics();

  config->set_value("verbose", d_->verbose,
                    "If true, write status messages to the terminal showing "
                    "debugging information");

  config->set_value("base_camera:focal_length", K.focal_length(),
                    "focal length of the base camera model");

  config->set_value("base_camera:principal_point", K.principal_point(),
                    "The principal point of the base camera model \"x y\".\n"
                    "It is usually safe to assume this is the center of the "
                    "image.");

  config->set_value("base_camera:aspect_ratio", K.aspect_ratio(),
                    "the pixel aspect ratio of the base camera model");

  config->set_value("base_camera:skew", K.skew(),
                    "The skew factor of the base camera model.\n"
                    "This is almost always zero in any real camera.");

  // nested algorithm configurations
  algo::estimate_essential_matrix
      ::get_nested_algo_configuration("essential_mat_estimator",
                                      config, d_->e_estimator);
  return config;
}


/// Set this algorithm's properties via a config block
void
initialize_cameras_landmarks
::set_configuration(config_block_sptr config)
{
  const camera_intrinsics_d& K = d_->base_camera.get_intrinsics();

  // Set nested algorithm configurations
  algo::estimate_essential_matrix
      ::set_nested_algo_configuration("essential_mat_estimator",
                                      config, d_->e_estimator);

  d_->verbose = config->get_value<bool>("verbose",
                                        d_->verbose);

  config_block_sptr bc = config->subblock("base_camera");
  camera_intrinsics_d K2(bc->get_value<double>("focal_length",
                                               K.focal_length()),
                         bc->get_value<vector_2d>("principal_point",
                                                  K.principal_point()),
                         bc->get_value<double>("aspect_ratio",
                                               K.aspect_ratio()),
                         bc->get_value<double>("skew",
                                               K.skew()));
  d_->base_camera.set_intrinsics(K2);
}


/// Check that the algorithm's currently configuration is valid
bool
initialize_cameras_landmarks
::check_configuration(config_block_sptr config) const
{
  return algo::estimate_essential_matrix
             ::check_nested_algo_configuration("essential_mat_estimator",
                                               config);
}


/// Initialize the camera and landmark parameters given a set of tracks
void
initialize_cameras_landmarks
::initialize(camera_map_sptr& cameras,
             landmark_map_sptr& landmarks,
             track_set_sptr tracks) const
{
  if( !cameras || !landmarks || !tracks )
  {
    throw invalid_value("Some required input data is NULL.");
  }
  if( !d_->e_estimator )
  {
    throw invalid_value("Essential matrix estimator not initialized.");
  }
  typedef maptk::camera_map::map_camera_t map_cam_t;
  typedef maptk::landmark_map::map_landmark_t map_landmark_t;

  // extract data from containers
  map_cam_t cams = cameras->cameras();
  map_landmark_t lms = landmarks->landmarks();
  std::vector<track_sptr> trks = tracks->tracks();

  //
  // Find the set of all frame numbers containing a camera and track data
  //

  // find the set of all frame IDs covered by these tracks
  std::set<frame_id_t> track_frame_ids = tracks->all_frame_ids();

  // find the set of all track IDs in this track set
  std::set<track_id_t> track_lm_ids = tracks->all_track_ids();

  std::set<frame_id_t> cam_frame_ids;
  std::deque<frame_id_t> new_frame_ids;
  BOOST_FOREACH(const map_cam_t::value_type& p, cams)
  {
    cam_frame_ids.insert(p.first);
  }
  std::set_difference(track_frame_ids.begin(), track_frame_ids.end(),
                      cam_frame_ids.begin(), cam_frame_ids.end(),
                      std::back_inserter(new_frame_ids));

  if(new_frame_ids.empty())
  {
    // no new frames to initialize cameras on
    return;
  }

  if(cams.empty())
  {
    // first frame, initilze to base camera
    frame_id_t f = new_frame_ids.front();
    new_frame_ids.pop_front();
    cams[f] = camera_sptr(new camera_d(d_->base_camera));
  }

  frame_id_t last_frame = cams.rbegin()->first;
  while( !new_frame_ids.empty() )
  {
    frame_id_t f = new_frame_ids.front();
    new_frame_ids.pop_front();
    // get the subset of tracks that have features on frame f
    track_set_sptr ftracks = tracks->active_tracks(static_cast<int>(f));
    // get the subset of tracks that also  have features on the last frame
    ftracks = ftracks->active_tracks(static_cast<int>(last_frame));

    // find existing landmarks for these tracks
    map_landmark_t flms;
    std::vector<track_sptr> trks = ftracks->tracks();
    BOOST_FOREACH(const track_sptr& t, trks)
    {
      map_landmark_t::const_iterator li = lms.find(t->id());
      if( li != lms.end() )
      {
        flms.insert(*li);
      }
    }

    cams[f] = d_->init_camera(f, last_frame, cams, trks, flms);

    last_frame = f;
  }

  cameras = camera_map_sptr(new simple_camera_map(cams));
}


} // end namespace vxl

} // end namespace maptk
