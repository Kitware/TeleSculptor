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
 * \brief Implementation of core camera and landmark initialization algorithm
 */

#include "initialize_cameras_landmarks.h"

#include <deque>

#include <boost/foreach.hpp>

#include <vital/exceptions.h>
#include <vital/io/eigen_io.h>

#include <vital/algo/estimate_essential_matrix.h>
#include <vital/algo/triangulate_landmarks.h>

#include <maptk/triangulate.h>

using namespace kwiver::vital;

namespace kwiver {
namespace maptk {

namespace core
{


/// Private implementation class
class initialize_cameras_landmarks::priv
{
public:
  /// Constructor
  priv()
  : verbose(false),
    retriangulate_all(false),
    base_camera()
  {
  }

  priv(const priv& other)
  : verbose(other.verbose),
    retriangulate_all(other.retriangulate_all),
    base_camera(other.base_camera)
  {
  }

  /// Construct and initialized camera for \a frame
  camera_sptr init_camera(frame_id_t frame, frame_id_t last_frame,
                          const camera_map::map_camera_t& cams,
                          const std::vector<track_sptr>& trks,
                          const landmark_map::map_landmark_t& lms) const;

  /// Re-triangulate all landmarks for provided tracks
  void retriangulate(landmark_map::map_landmark_t& lms,
                     const camera_map::map_camera_t& cams,
                     const std::vector<track_sptr>& trks,
                     const std::set<landmark_id_t>& new_lm_ids) const;

  /// Estimate the translation scale using a 2d-3d correspondence
  double estimate_t_scale(const vector_3d& KRp,
                          const vector_3d& Kt,
                          const vector_2d& pt2d) const;

  /// Compute a valid left camera from an essential matrix
  /**
   *  There for four valid left camera possibilities for any essential
   *  matrix (assuming the right camera is the identity camera).
   *  This function select the left camera such that a corresponding
   *  pair of points triangulates in front of both cameras.
   */
  camera_d
  extract_valid_left_camera(const essential_matrix_d& e,
                            const vector_2d& left_pt,
                            const vector_2d& right_pt) const;

  bool verbose;
  bool retriangulate_all;
  camera_d base_camera;
  vital::algo::estimate_essential_matrix_sptr e_estimator;
  vital::algo::triangulate_landmarks_sptr lm_triangulator;
};


/// Construct and initialized camera for \a frame
camera_sptr
initialize_cameras_landmarks::priv
::init_camera(frame_id_t frame, frame_id_t last_frame,
              const camera_map::map_camera_t& cams,
              const std::vector<track_sptr>& trks,
              const landmark_map::map_landmark_t& lms) const
{
  typedef landmark_map::map_landmark_t lm_map_t;
  // extract coresponding image points and landmarks
  std::vector<vector_2d> pts_right, pts_left;
  std::vector<landmark_sptr> pts_lm;
  for(unsigned int i=0; i<trks.size(); ++i)
  {
    pts_right.push_back(trks[i]->find(last_frame)->feat->loc());
    pts_left.push_back(trks[i]->find(frame)->feat->loc());
    lm_map_t::const_iterator li = lms.find(trks[i]->id());
    if( li != lms.end() )
    {
      pts_lm.push_back(li->second);
    }
    else
    {
      pts_lm.push_back(landmark_sptr());
    }
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
  essential_matrix_sptr E_sptr = e_estimator->estimate(pts_right, pts_left,
                                                       cal_right, cal_left,
                                                       inliers, 2.0);
  const essential_matrix_d E(*E_sptr);

  unsigned num_inliers = static_cast<unsigned>(std::count(inliers.begin(),
                                                          inliers.end(), true));
  if( this->verbose )
  {
    std::cout << "E matrix num inliers = "<<num_inliers<<std::endl;
  }

  // get the first inlier index
  unsigned int inlier_idx = 0;
  for(; inlier_idx < inliers.size() && !inliers[inlier_idx]; ++inlier_idx);

  // get the first inlier correspondence to
  // disambiguate essential matrix solutions
  vector_2d left_pt = cal_left.unmap(pts_left[inlier_idx]);
  vector_2d right_pt = cal_right.unmap(pts_right[inlier_idx]);

  // compute the corresponding camera rotation and translation (up to scale)
  camera_d cam = extract_valid_left_camera(E, left_pt, right_pt);
  cam.set_intrinsics(cal_left);

  // compute the scale from existing landmark locations (if available)
  matrix_3x3d prev_R(prev_cam->rotation());
  vector_3d prev_t = prev_cam->translation();
  matrix_3x3d K(cam.get_intrinsics());
  matrix_3x3d KR = K * matrix_3x3d(cam.get_rotation());
  vector_3d Kt = K * cam.get_translation();
  std::vector<double> scales;
  scales.reserve(num_inliers);
  for(unsigned int i=0; i<inliers.size(); ++i)
  {
    if( !inliers[i] || !pts_lm[i] )
    {
      continue;
    }
    vector_3d pt3d = prev_R * pts_lm[i]->loc() + prev_t;
    const vector_2d& pt2d = pts_left[i];
    scales.push_back(estimate_t_scale(KR*pt3d, Kt, pt2d));
  }
  // find the median scale
  double median_scale = 1.0;
  if( !scales.empty() )
  {
    size_t n = scales.size() / 2;
    std::nth_element(scales.begin(), scales.begin()+n, scales.end());
    median_scale = scales[n];
  }
  if( this->verbose )
  {
    std::cout << " median scale = "<< median_scale<<std::endl;
    if( !scales.empty() )
    {
      std::sort(scales.begin(), scales.end());
      std::cout << "    min scale = " << scales.front() << '\n'
                << "    max scale = " << scales.back() << std::endl;
    }
  }

  // adjust pose relative to the previous camera
  vector_3d new_t = cam.get_rotation() * prev_cam->translation()
                  + median_scale * cam.translation();
  cam.set_rotation(cam.get_rotation() * prev_cam->rotation());
  cam.set_translation(new_t);

  return camera_sptr(new camera_d(cam));
}


/// Re-triangulate all landmarks for provided tracks
void
initialize_cameras_landmarks::priv
::retriangulate(landmark_map::map_landmark_t& lms,
                const camera_map::map_camera_t& cams,
                const std::vector<track_sptr>& trks,
                const std::set<landmark_id_t>& new_lm_ids) const
{
  typedef landmark_map::map_landmark_t lm_map_t;
  lm_map_t init_lms;
  BOOST_FOREACH(const track_sptr& t, trks)
  {
    const track_id_t& tid = t->id();
    if( !this->retriangulate_all &&
        new_lm_ids.find(tid) == new_lm_ids.end() )
    {
      continue;
    }
    lm_map_t::const_iterator li = lms.find(tid);
    if( li == lms.end() )
    {
      landmark_sptr lm(new landmark_d(vector_3d(0,0,0)));
      init_lms[static_cast<landmark_id_t>(tid)] = lm;
    }
    else
    {
      init_lms.insert(*li);
    }
  }

  landmark_map_sptr lm_map(new simple_landmark_map(init_lms));
  camera_map_sptr cam_map(new simple_camera_map(cams));
  track_set_sptr tracks(new simple_track_set(trks));
  this->lm_triangulator->triangulate(cam_map, tracks, lm_map);

  BOOST_FOREACH(const lm_map_t::value_type& p, lm_map->landmarks())
  {
    lms[p.first] = p.second;
  }
}


/// Estimate the translation scale using a 2d-3d correspondence
double
initialize_cameras_landmarks::priv
::estimate_t_scale(const vector_3d& KRp,
                   const vector_3d& Kt,
                   const vector_2d& pt2d) const
{
  vector_3d a = KRp;
  vector_3d b = Kt;
  a.x() = pt2d.x() * a.z() - a.x();
  b.x() = pt2d.x() * b.z() - b.x();
  a.y() = pt2d.y() * a.z() - a.y();
  b.y() = pt2d.y() * b.z() - b.y();
  double cx = a.x()*b.z() - a.z()*b.x();
  double cy = a.y()*b.z() - a.z()*b.y();
  return (a.x()*cx + a.y()*cy) / -(b.x()*cx + b.y()*cy);
}

/// Compute a valid left camera from an essential matrix
camera_d
initialize_cameras_landmarks::priv
::extract_valid_left_camera(const essential_matrix_d& e,
                            const vector_2d& left_pt,
                            const vector_2d& right_pt) const
{
  /// construct an identity right camera
  const vector_3d t = e.translation();
  rotation_d R = e.rotation();

  std::vector<vector_2d> pts;
  pts.push_back(right_pt);
  pts.push_back(left_pt);

  std::vector<camera_d> cams(2);
  const camera_d& left_camera = cams[1];

  // option 1
  cams[1] = camera_d(R.inverse()*-t, R);
  vector_3d pt3 = triangulate_inhomog(cams, pts);
  if( pt3.z() > 0.0 && left_camera.depth(pt3) > 0.0 )
  {
    return left_camera;
  }

  // option 2, with negated translation
  cams[1] = camera_d(R.inverse()*t, R);
  pt3 = triangulate_inhomog(cams, pts);
  if( pt3.z() > 0.0 && left_camera.depth(pt3) > 0.0 )
  {
    return left_camera;
  }

  // option 3, with the twisted pair rotation
  R = e.twisted_rotation();
  cams[1] = camera_d(R.inverse()*-t, R);
  pt3 = triangulate_inhomog(cams, pts);
  if( pt3.z() > 0.0 && left_camera.depth(pt3) > 0.0 )
  {
    return left_camera;
  }

  // option 4, with negated translation
  cams[1] = camera_d(R.inverse()*t, R);
  pt3 = triangulate_inhomog(cams, pts);
  if( pt3.z() > 0.0 && left_camera.depth(pt3) > 0.0 )
  {
    return left_camera;
  }
  // should never get here
  return camera_d();
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


/// Get this algorithm's \link vital::config_block configuration block \endlink
vital::config_block_sptr
initialize_cameras_landmarks
::get_configuration() const
{
  // get base config from base class
  vital::config_block_sptr config =
      vital::algo::initialize_cameras_landmarks::get_configuration();

  const camera_intrinsics_d& K = d_->base_camera.get_intrinsics();

  config->set_value("verbose", d_->verbose,
                    "If true, write status messages to the terminal showing "
                    "debugging information");

  config->set_value("retriangulate_all", d_->retriangulate_all,
                    "If true, re-triangulate all landmarks observed by a newly "
                    "initialized camera.  Otherwise, only triangulate or "
                    "re-triangulate landmarks that are marked for initialization.");

  config->set_value("base_camera:focal_length", K.focal_length(),
                    "focal length of the base camera model");

  config->set_value("base_camera:principal_point", K.principal_point().transpose(),
                    "The principal point of the base camera model \"x y\".\n"
                    "It is usually safe to assume this is the center of the "
                    "image.");

  config->set_value("base_camera:aspect_ratio", K.aspect_ratio(),
                    "the pixel aspect ratio of the base camera model");

  config->set_value("base_camera:skew", K.skew(),
                    "The skew factor of the base camera model.\n"
                    "This is almost always zero in any real camera.");

  // nested algorithm configurations
  vital::algo::estimate_essential_matrix
      ::get_nested_algo_configuration("essential_mat_estimator",
                                      config, d_->e_estimator);
  vital::algo::triangulate_landmarks
      ::get_nested_algo_configuration("lm_triangulator",
                                      config, d_->lm_triangulator);
  return config;
}


/// Set this algorithm's properties via a config block
void
initialize_cameras_landmarks
::set_configuration(vital::config_block_sptr config)
{
  const camera_intrinsics_d& K = d_->base_camera.get_intrinsics();

  // Set nested algorithm configurations
  vital::algo::estimate_essential_matrix
      ::set_nested_algo_configuration("essential_mat_estimator",
                                      config, d_->e_estimator);
  vital::algo::triangulate_landmarks
      ::set_nested_algo_configuration("lm_triangulator",
                                      config, d_->lm_triangulator);

  d_->verbose = config->get_value<bool>("verbose",
                                        d_->verbose);

  d_->retriangulate_all = config->get_value<bool>("retriangulate_all",
                                                  d_->retriangulate_all);

  vital::config_block_sptr bc = config->subblock("base_camera");
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
::check_configuration(vital::config_block_sptr config) const
{
  return vital::algo::estimate_essential_matrix
             ::check_nested_algo_configuration("essential_mat_estimator",
                                               config)
      && vital::algo::triangulate_landmarks
             ::check_nested_algo_configuration("lm_triangulator", config);
}

namespace
{

/// Extract valid cameras and cameras to initialize.
/**
 * If \a cameras is NULL then return empty cam_map and frame_ids unchanged.
 * If not NULL, return frame_ids containing IDs of all NULL cameras and
 * return cam_map containing all valid cameras.
 * \param [in]     cameras the camera map object to extract from
 * \param [in,out] frame_ids the set of all frames (input),
 *                           the set of frame to initialize (output)
 * \param [out]    cam_map the extract map of valid camera
 */
void extract_cameras(const camera_map_sptr& cameras,
                     std::set<frame_id_t>& frame_ids,
                     camera_map::map_camera_t& cam_map)
{
  cam_map.clear();
  if( !cameras )
  {
    return;
  }

  typedef camera_map::map_camera_t map_cam_t;
  map_cam_t all_cams = cameras->cameras();

  // Find the set of all cameras that need to be initialized
  std::set<frame_id_t> new_frames;
  BOOST_FOREACH(const map_cam_t::value_type& p, all_cams)
  {
    if(p.second)
    {
      cam_map.insert(p);
    }
    else if( frame_ids.count(p.first) )
    {
      new_frames.insert(p.first);
    }
  }
  frame_ids = new_frames;
}


/// Extract valid landmarks and landmarks to initialize.
/**
 * If \a landmarks is NULL then return empty lm_map and track_ids unchanged.
 * If not NULL, return track_ids containing IDs of all NULL landmark and
 * return lm_map containing all valid landmarks.
 * \param [in]     landmarks the landmark map object to extract from
 * \param [in,out] track_ids the set of all tracks (input),
 *                           the set of landmarks to initialize (output)
 * \param [out]    lm_map the extract map of valid landmarks
 */
void extract_landmarks(const landmark_map_sptr& landmarks,
                       std::set<track_id_t>& track_ids,
                       landmark_map::map_landmark_t& lm_map)
{
  lm_map.clear();
  if( !landmarks )
  {
    return;
  }

  typedef landmark_map::map_landmark_t map_landmark_t;
  map_landmark_t all_lms = landmarks->landmarks();

  // Find the set of all landmarks that need to be initialized
  std::set<track_id_t> new_landmarks;
  BOOST_FOREACH(const map_landmark_t::value_type& p, all_lms)
  {
    if(p.second)
    {
      lm_map.insert(p);
    }
    else if( track_ids.count(p.first) )
    {
      new_landmarks.insert(p.first);
    }
  }
  track_ids = new_landmarks;
}

} // end anonymous namespace


/// Initialize the camera and landmark parameters given a set of tracks
void
initialize_cameras_landmarks
::initialize(camera_map_sptr& cameras,
             landmark_map_sptr& landmarks,
             track_set_sptr tracks) const
{
  if( !tracks )
  {
    throw invalid_value("Some required input data is NULL.");
  }
  if( !d_->e_estimator )
  {
    throw invalid_value("Essential matrix estimator not initialized.");
  }
  if( !d_->lm_triangulator )
  {
    throw invalid_value("Landmark triangulator not initialized.");
  }
  typedef vital::camera_map::map_camera_t map_cam_t;
  typedef vital::landmark_map::map_landmark_t map_landmark_t;

  // Extract the existing cameras and camera ids to be initialized
  std::set<frame_id_t> frame_ids = tracks->all_frame_ids();
  map_cam_t cams;
  extract_cameras(cameras, frame_ids, cams);
  std::deque<frame_id_t> new_frame_ids(frame_ids.begin(), frame_ids.end());

  // Extract the existing landmarks and landmark ids to be initialized
  std::set<track_id_t> track_ids = tracks->all_track_ids();
  map_landmark_t lms;
  extract_landmarks(landmarks, track_ids, lms);
  std::set<landmark_id_t> new_lm_ids(track_ids.begin(), track_ids.end());

  std::vector<track_sptr> trks = tracks->tracks();

  if(new_frame_ids.empty() && new_lm_ids.empty())
  {
    //nothing to initialize
    return;
  }

  // initialize landmarks if there are already at least two cameras
  if(cams.size() > 2 && !new_lm_ids.empty())
  {
    map_landmark_t init_lms;
    BOOST_FOREACH(const landmark_id_t& lmid, new_lm_ids)
    {
      landmark_sptr lm(new landmark_d(vector_3d(0,0,0)));
      init_lms[static_cast<landmark_id_t>(lmid)] = lm;
    }

    landmark_map_sptr lm_map(new simple_landmark_map(init_lms));
    camera_map_sptr cam_map(new simple_camera_map(cams));
    d_->lm_triangulator->triangulate(cam_map, tracks, lm_map);

    BOOST_FOREACH(const map_landmark_t::value_type& p, lm_map->landmarks())
    {
      lms[p.first] = p.second;
    }
  }

  if(cams.empty())
  {
    // first frame, initilze to base camera
    frame_id_t f = new_frame_ids.front();
    new_frame_ids.pop_front();
    cams[f] = camera_sptr(new camera_d(d_->base_camera));
  }

  frame_id_t other_frame = cams.rbegin()->first;
  while( !new_frame_ids.empty() )
  {
    frame_id_t f = new_frame_ids.front();
    new_frame_ids.pop_front();

    // get the closest frame number with an exisiting camera
    map_cam_t::const_iterator ci = cams.lower_bound(f);
    if( ci == cams.end() )
    {
      other_frame = cams.rbegin()->first;
    }
    else
    {
      other_frame = ci->first;
      if (ci != cams.begin() &&
          (other_frame-f) >= (f-(--ci)->first))
      {
        other_frame = ci->first;
      }
    }
    if(d_->verbose)
    {
      std::cout << f << " uses reference "<< other_frame <<std::endl;
    }

    // get the subset of tracks that have features on frame f
    track_set_sptr ftracks = tracks->active_tracks(static_cast<int>(f));
    // get the subset of tracks that also  have features on the last frame
    ftracks = ftracks->active_tracks(static_cast<int>(other_frame));

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

    cams[f] = d_->init_camera(f, other_frame, cams, trks, flms);

    // triangulate (or re-triangulate) points seen by the new camera
    d_->retriangulate(lms, cams, trks, new_lm_ids);

    if(d_->verbose)
    {
      std::cout << "frame "<<f<<" - num landmarks = "<< lms.size() << std::endl;
    }
  }

  cameras = camera_map_sptr(new simple_camera_map(cams));
  landmarks = landmark_map_sptr(new simple_landmark_map(lms));
}


} // end namespace core

} // end namespace maptk
} // end namespace kwiver
