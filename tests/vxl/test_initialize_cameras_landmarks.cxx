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

#include <test_common.h>
#include <test_math.h>
#include <test_scene.h>

#include <maptk/core/projected_track_set.h>
#include <maptk/core/metrics.h>
#include <maptk/core/similarity.h>
#include <maptk/core/transform.h>
#include <maptk/vxl/register.h>
#include <maptk/vxl/estimate_essential_matrix.h>
#include <maptk/vxl/estimate_similarity_transform.h>
#include <maptk/vxl/initialize_cameras_landmarks.h>

#include <boost/foreach.hpp>


#define TEST_ARGS ()

DECLARE_TEST_MAP();

int
main(int argc, char* argv[])
{
  CHECK_ARGS(1);

  maptk::vxl::register_algorithms();

  testname_t const testname = argv[1];

  RUN_TEST(testname);
}


IMPLEMENT_TEST(create)
{
  using namespace maptk;
  algo::initialize_cameras_landmarks_sptr init = algo::initialize_cameras_landmarks::create("vxl");
  if (!init)
  {
    TEST_ERROR("Unable to create vxl::initialize_cameras_landmarks by name");
  }
}


// helper function to configure the algorithm
void configure_algo(maptk::vxl::initialize_cameras_landmarks& algo,
                    const maptk::camera_intrinsics_d& K)
{
  using namespace maptk;
  config_block_sptr cfg = algo.get_configuration();
  cfg->set_value("verbose", "true");
  cfg->set_value("base_camera:focal_length", K.focal_length());
  cfg->set_value("base_camera:principal_point", K.principal_point());
  cfg->set_value("base_camera:aspect_ratio", K.aspect_ratio());
  cfg->set_value("base_camera:skew", K.skew());
  cfg->set_value("essential_mat_estimator:type", "vxl");
  cfg->set_value("essential_mat_estimator:vxl:num_ransac_samples", 10);
  cfg->set_value("lm_triangulator:type", "vxl");
  algo.set_configuration(cfg);

  if(!algo.check_configuration(cfg))
  {
    std::cout << "Error: configuration is not correct" << std::endl;
    return;
  }
}


// test initialization with ideal points
IMPLEMENT_TEST(ideal_points)
{
  using namespace maptk;
  vxl::initialize_cameras_landmarks init;

  // create landmarks at the random locations
  landmark_map_sptr landmarks = testing::init_landmarks(100);
  landmarks = testing::noisy_landmarks(landmarks, 1.0);

  // create a camera sequence (elliptical path)
  camera_map_sptr cameras = testing::camera_seq();

  // create tracks from the projections
  track_set_sptr tracks = projected_tracks(landmarks, cameras);

  camera_intrinsics_d K = cameras->cameras()[0]->intrinsics();
  configure_algo(init, K);

  camera_map_sptr new_cameras;
  landmark_map_sptr new_landmarks;
  init.initialize(new_cameras, new_landmarks, tracks);

  camera_map::map_camera_t orig_cams = cameras->cameras();
  camera_map::map_camera_t new_cams = new_cameras->cameras();

  typedef algo::estimate_similarity_transform_sptr est_sim_sptr;
  est_sim_sptr est_sim(new vxl::estimate_similarity_transform());
  similarity_d global_sim = est_sim->estimate_transform(new_cameras, cameras);
  std::cout << "similarity = "<<global_sim<<std::endl;


  BOOST_FOREACH(camera_map::map_camera_t::value_type p, orig_cams)
  {
    camera_sptr new_cam_t = transform(new_cams[p.first], global_sim);
    rotation_d dR = new_cam_t->rotation().inverse() * p.second->rotation();
    TEST_NEAR("rotation difference magnitude", dR.angle(), 0.0, 1e-8);

    double dt = (p.second->center() - new_cam_t->center()).magnitude();
    TEST_NEAR("camera center difference", dt, 0.0, 1e-8);
  }

  landmark_map::map_landmark_t orig_lms = landmarks->landmarks();
  landmark_map::map_landmark_t new_lms = new_landmarks->landmarks();
  BOOST_FOREACH(landmark_map::map_landmark_t::value_type p, orig_lms)
  {
    landmark_sptr new_lm_tr = transform(new_lms[p.first], global_sim);

    double dt = (p.second->loc() - new_lm_tr->loc()).magnitude();
    TEST_NEAR("landmark location difference", dt, 0.0, 1e-8);
  }
}


// test initialization with noisy points
IMPLEMENT_TEST(noisy_points)
{
  using namespace maptk;
  vxl::initialize_cameras_landmarks init;

  // create landmarks at the random locations
  landmark_map_sptr landmarks = testing::init_landmarks(100);
  landmarks = testing::noisy_landmarks(landmarks, 1.0);

  // create a camera sequence (elliptical path)
  camera_map_sptr cameras = testing::camera_seq();

  // create tracks from the projections
  track_set_sptr tracks = projected_tracks(landmarks, cameras);

  // add random noise to track image locations
  tracks = testing::noisy_tracks(tracks, 0.5);

  camera_intrinsics_d K = cameras->cameras()[0]->intrinsics();
  configure_algo(init, K);

  camera_map_sptr new_cameras;
  landmark_map_sptr new_landmarks;
  init.initialize(new_cameras, new_landmarks, tracks);

  camera_map::map_camera_t orig_cams = cameras->cameras();
  camera_map::map_camera_t new_cams = new_cameras->cameras();

  typedef algo::estimate_similarity_transform_sptr est_sim_sptr;
  est_sim_sptr est_sim(new vxl::estimate_similarity_transform());
  similarity_d global_sim = est_sim->estimate_transform(new_cameras, cameras);
  std::cout << "similarity = "<<global_sim<<std::endl;


  BOOST_FOREACH(camera_map::map_camera_t::value_type p, orig_cams)
  {
    camera_sptr new_cam_t = transform(new_cams[p.first], global_sim);
    rotation_d dR = new_cam_t->rotation().inverse() * p.second->rotation();
    TEST_NEAR("rotation difference magnitude", dR.angle(), 0.0, 0.1);

    double dt = (p.second->center() - new_cam_t->center()).magnitude();
    TEST_NEAR("camera center difference", dt, 0.0, 0.1);
  }

  landmark_map::map_landmark_t orig_lms = landmarks->landmarks();
  landmark_map::map_landmark_t new_lms = new_landmarks->landmarks();
  BOOST_FOREACH(landmark_map::map_landmark_t::value_type p, orig_lms)
  {
    landmark_sptr new_lm_tr = transform(new_lms[p.first], global_sim);

    double dt = (p.second->loc() - new_lm_tr->loc()).magnitude();
    TEST_NEAR("landmark location difference", dt, 0.0, 0.1);
  }
}


// test initialization with subsets of cameras and landmarks
IMPLEMENT_TEST(subset_init)
{
  using namespace maptk;
  vxl::initialize_cameras_landmarks init;

  // create landmarks at the random locations
  landmark_map_sptr landmarks = testing::init_landmarks(100);
  landmarks = testing::noisy_landmarks(landmarks, 1.0);

  // create a camera sequence (elliptical path)
  camera_map_sptr cameras = testing::camera_seq();

  // create tracks from the projections
  track_set_sptr tracks = projected_tracks(landmarks, cameras);

  camera_intrinsics_d K = cameras->cameras()[0]->intrinsics();
  configure_algo(init, K);

  // mark every 3rd camera for initialization
  camera_map::map_camera_t cams_to_init;
  for(unsigned int i=0; i<cameras->size(); ++i)
  {
    if( i%3 == 0 )
    {
      cams_to_init[i] = camera_sptr();
    }
  }
  camera_map_sptr new_cameras(new simple_camera_map(cams_to_init));

  // mark every 5rd landmark for initialization
  landmark_map::map_landmark_t lms_to_init;
  for(unsigned int i=0; i<landmarks->size(); ++i)
  {
    if( i%5 == 0 )
    {
      lms_to_init[i] = landmark_sptr();
    }
  }
  landmark_map_sptr new_landmarks(new simple_landmark_map(lms_to_init));

  init.initialize(new_cameras, new_landmarks, tracks);

  // test that we only initialized the requested objects
  BOOST_FOREACH(camera_map::map_camera_t::value_type p, new_cameras->cameras())
  {
    TEST_EQUAL("Camera initialized", bool(p.second), true);
    TEST_EQUAL("Expected camera id", p.first % 3, 0);
  }
  BOOST_FOREACH(landmark_map::map_landmark_t::value_type p, new_landmarks->landmarks())
  {
    TEST_EQUAL("Landmark initialized", bool(p.second), true);
    TEST_EQUAL("Expected landmark id", p.first % 5, 0);
  }

  // calling this again should do nothing
  camera_map_sptr before_cameras = new_cameras;
  landmark_map_sptr before_landmarks = new_landmarks;
  init.initialize(new_cameras, new_landmarks, tracks);
  TEST_EQUAL("Initialization No-Op on cameras", before_cameras, new_cameras);
  TEST_EQUAL("Initialization No-Op on landmarks", before_landmarks, new_landmarks);

  // add the rest of the cameras
  cams_to_init = new_cameras->cameras();
  for(unsigned int i=0; i<cameras->size(); ++i)
  {
    if( cams_to_init.find(i) == cams_to_init.end() )
    {
      cams_to_init[i] = camera_sptr();
    }
  }
  new_cameras = camera_map_sptr(new simple_camera_map(cams_to_init));

  // add the rest of the landmarks
  lms_to_init = new_landmarks->landmarks();
  for(unsigned int i=0; i<landmarks->size(); ++i)
  {
    if( lms_to_init.find(i) == lms_to_init.end() )
    {
      lms_to_init[i] = landmark_sptr();
    }
  }
  new_landmarks = landmark_map_sptr(new simple_landmark_map(lms_to_init));

  // initialize the rest
  init.initialize(new_cameras, new_landmarks, tracks);

  camera_map::map_camera_t orig_cams = cameras->cameras();
  camera_map::map_camera_t new_cams = new_cameras->cameras();

  typedef algo::estimate_similarity_transform_sptr est_sim_sptr;
  est_sim_sptr est_sim(new vxl::estimate_similarity_transform());
  similarity_d global_sim = est_sim->estimate_transform(new_cameras, cameras);
  std::cout << "similarity = "<<global_sim<<std::endl;


  BOOST_FOREACH(camera_map::map_camera_t::value_type p, orig_cams)
  {
    camera_sptr new_cam_t = transform(new_cams[p.first], global_sim);
    rotation_d dR = new_cam_t->rotation().inverse() * p.second->rotation();
    TEST_NEAR("rotation difference magnitude", dR.angle(), 0.0, 1e-8);

    double dt = (p.second->center() - new_cam_t->center()).magnitude();
    TEST_NEAR("camera center difference", dt, 0.0, 1e-8);
  }

  landmark_map::map_landmark_t orig_lms = landmarks->landmarks();
  landmark_map::map_landmark_t new_lms = new_landmarks->landmarks();
  BOOST_FOREACH(landmark_map::map_landmark_t::value_type p, orig_lms)
  {
    landmark_sptr new_lm_tr = transform(new_lms[p.first], global_sim);

    double dt = (p.second->loc() - new_lm_tr->loc()).magnitude();
    TEST_NEAR("landmark location difference", dt, 0.0, 1e-8);
  }
}
