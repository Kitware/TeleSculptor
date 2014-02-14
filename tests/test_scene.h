/*ckwg +5
 * Copyright 2011-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 *
 * \brief Various functions for creating a simple SBA test scene
 *
 * These functions are based on MAPTK core and shared by various tests
 */

#ifndef MAPTK_TEST_TEST_SCENE_H_
#define MAPTK_TEST_TEST_SCENE_H_

#include <test_random_point.h>

#include <maptk/core/camera_map.h>
#include <maptk/core/landmark_map.h>
#include <maptk/core/track_set.h>
#include <boost/foreach.hpp>


// construct a map of landmarks at the corners of a cube centered at c
// with a side length of s
maptk::landmark_map_sptr
cube_corners(double s, const maptk::vector_3d& c=maptk::vector_3d(0,0,0))
{
  using namespace maptk;

  // create corners of a cube
  landmark_map::map_landmark_t landmarks;
  s /= 2.0;
  landmarks[0] = landmark_sptr(new landmark_d(c + vector_3d(-s, -s, -s)));
  landmarks[1] = landmark_sptr(new landmark_d(c + vector_3d(-s, -s,  s)));
  landmarks[2] = landmark_sptr(new landmark_d(c + vector_3d(-s,  s, -s)));
  landmarks[3] = landmark_sptr(new landmark_d(c + vector_3d(-s,  s,  s)));
  landmarks[4] = landmark_sptr(new landmark_d(c + vector_3d( s, -s, -s)));
  landmarks[5] = landmark_sptr(new landmark_d(c + vector_3d( s, -s,  s)));
  landmarks[6] = landmark_sptr(new landmark_d(c + vector_3d( s,  s, -s)));
  landmarks[7] = landmark_sptr(new landmark_d(c + vector_3d( s,  s,  s)));

  return landmark_map_sptr(new simple_landmark_map(landmarks));
}


// construct map of landmarks will all locations at c
maptk::landmark_map_sptr
init_landmarks(maptk::landmark_id_t num_lm,
               const maptk::vector_3d& c=maptk::vector_3d(0,0,0))
{
  using namespace maptk;

  landmark_map::map_landmark_t lm_map;
  for (landmark_id_t i=0; i<num_lm; ++i)
  {
    lm_map[i] = landmark_sptr(new landmark_d(c));
  }
  return landmark_map_sptr(new simple_landmark_map(lm_map));
}


// add Gaussian noise to the landmark positions
maptk::landmark_map_sptr
noisy_landmarks(maptk::landmark_map_sptr landmarks,
                double stdev=1.0)
{
  using namespace maptk;

  landmark_map::map_landmark_t lm_map = landmarks->landmarks();
  BOOST_FOREACH(landmark_map::map_landmark_t::value_type& p, lm_map)
  {
    landmark_d& lm = dynamic_cast<landmark_d&>(*p.second);
    lm.set_loc(lm.get_loc() + random_point3(stdev));
  }
  return landmark_map_sptr(new simple_landmark_map(lm_map));
}


// create a camera sequence (elliptical path)
maptk::camera_map_sptr
camera_seq(maptk::frame_id_t num_cams = 20)
{
  using namespace maptk;
  camera_map::map_camera_t cameras;

  // create a camera sequence (elliptical path)
  camera_intrinsics_d K(1000, vector_2d(640,480));
  rotation_d R; // identity
  for (frame_id_t i=0; i<num_cams; ++i)
  {
    double frac = static_cast<double>(i) / num_cams;
    double x = 4 * std::cos(2*frac);
    double y = 3 * std::sin(2*frac);
    camera_d* cam = new camera_d(vector_3d(x,y,2+frac), R, K);
    // look at the origin
    cam->look_at(vector_3d(0,0,0));
    cameras[i] = camera_sptr(cam);
  }
  return camera_map_sptr(new simple_camera_map(cameras));
}


// create an initial camera sequence with all cameras at the same location
maptk::camera_map_sptr
init_cameras(maptk::frame_id_t num_cams = 20)
{
  using namespace maptk;
  camera_map::map_camera_t cameras;

  // create a camera sequence (elliptical path)
  camera_intrinsics_d K(1000, vector_2d(640,480));
  rotation_d R; // identity
  vector_3d c(0, 0, 1);
  for (frame_id_t i=0; i<num_cams; ++i)
  {
    camera_d* cam = new camera_d(c, R, K);
    // look at the origin
    cam->look_at(vector_3d(0,0,0), vector_3d(0,1,0));
    cameras[i] = camera_sptr(cam);
  }
  return camera_map_sptr(new simple_camera_map(cameras));
}


// add positional and rotational Gaussian noise to cameras
maptk::camera_map_sptr
noisy_cameras(maptk::camera_map_sptr cameras,
              double pos_stdev=1.0, double rot_stdev=1.0)
{
  using namespace maptk;

  camera_map::map_camera_t cam_map = cameras->cameras();
  BOOST_FOREACH(camera_map::map_camera_t::value_type& p, cam_map)
  {
    camera_d& cam = dynamic_cast<camera_d&>(*p.second);
    cam.set_center(cam.get_center() + random_point3(pos_stdev));
    rotation_d rand_rot(random_point3(rot_stdev));
    cam.set_rotation(cam.get_rotation() * rand_rot);
  }
  return camera_map_sptr(new simple_camera_map(cam_map));
}


// create tracks by projecting the landmarks into the cameras
maptk::track_set_sptr
projected_tracks(maptk::landmark_map_sptr landmarks,
                 maptk::camera_map_sptr cameras)
{
  using namespace maptk;
  std::vector<track_sptr> tracks;
  camera_map::map_camera_t cam_map = cameras->cameras();
  landmark_map::map_landmark_t lm_map = landmarks->landmarks();
  const track_id_t num_pts = static_cast<track_id_t>(landmarks->size());
  for (track_id_t i=0; i<num_pts; ++i)
  {
    track_sptr t(new track);
    t->set_id(i);
    tracks.push_back(t);
    BOOST_FOREACH(const camera_map::map_camera_t::value_type& p, cam_map)
    {
      const camera_d& cam = dynamic_cast<const camera_d&>(*p.second);
      feature_sptr f(new feature_d(cam.project(lm_map[i]->loc())));
      t->append(track::track_state(p.first, f, descriptor_sptr()));
    }
  }
  return track_set_sptr(new simple_track_set(tracks));
}


// randomly drop a fraction of the track states
maptk::track_set_sptr
subset_tracks(maptk::track_set_sptr in_tracks, double keep_frac=0.75)
{
  using namespace maptk;

  std::srand(0);
  std::vector<track_sptr> tracks = in_tracks->tracks();
  std::vector<track_sptr> new_tracks;
  const int rand_thresh = static_cast<int>(keep_frac * RAND_MAX);
  BOOST_FOREACH(const track_sptr& t, tracks)
  {
    track_sptr nt(new track);
    nt->set_id(t->id());
    std::cout << "track "<<t->id()<<":";
    for(track::history_const_itr it=t->begin(); it!=t->end(); ++it)
    {
      if(std::rand() < rand_thresh)
      {
        nt->append(*it);
        std::cout << " .";
      }
      else
      {
        std::cout << " X";
      }
    }
    std::cout << std::endl;
    new_tracks.push_back(nt);
  }
  return track_set_sptr(new simple_track_set(new_tracks));
}


// add Gaussian noise to track feature locations
maptk::track_set_sptr
noisy_tracks(maptk::track_set_sptr in_tracks, double stdev=1.0)
{
  using namespace maptk;

  std::vector<track_sptr> tracks = in_tracks->tracks();
  std::vector<track_sptr> new_tracks;
  BOOST_FOREACH(const track_sptr& t, tracks)
  {
    track_sptr nt(new track);
    nt->set_id(t->id());
    for(track::history_const_itr it=t->begin(); it!=t->end(); ++it)
    {
      vector_2d loc = it->feat->loc() + random_point2(stdev);
      track::track_state ts(*it);
      ts.feat = feature_sptr(new feature_d(loc));
      nt->append(ts);
    }
    new_tracks.push_back(nt);
  }
  return track_set_sptr(new simple_track_set(new_tracks));
}


#endif // MAPTK_TEST_TEST_SCENE_H_
