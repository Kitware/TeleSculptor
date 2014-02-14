/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include <test_common.h>
#include <test_scene.h>

#include <maptk/core/metrics.h>
#include <maptk/vxl/register.h>
#include <maptk/vxl/triangulate_landmarks.h>
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
  algo::triangulate_landmarks_sptr tri_lm = algo::triangulate_landmarks::create("vxl");
  if (!tri_lm)
  {
    TEST_ERROR("Unable to create vxl::triangulate_landmarks by name");
  }
}


// input to triangulation is the ideal solution, make sure it doesn't diverge
IMPLEMENT_TEST(from_solution)
{
  using namespace maptk;
  vxl::triangulate_landmarks tri_lm;

  // create landmarks at the corners of a cube
  landmark_map_sptr landmarks = cube_corners(2.0);

  // create a camera sequence (elliptical path)
  camera_map_sptr cameras = camera_seq();

  // create tracks from the projections
  track_set_sptr tracks = projected_tracks(landmarks, cameras);

  double init_rmse = reprojection_rmse(cameras->cameras(),
                                       landmarks->landmarks(),
                                       tracks->tracks());
  std::cout << "initial reprojection RMSE: " << init_rmse << std::endl;
  if (init_rmse > 1e-12)
  {
    TEST_ERROR("Initial reprojection RMSE should be small");
  }

  tri_lm.triangulate(cameras, tracks, landmarks);

  double end_rmse = reprojection_rmse(cameras->cameras(),
                                      landmarks->landmarks(),
                                      tracks->tracks());
  TEST_NEAR("RMSE after triangulation", end_rmse, 0.0, 1e-12);
}


// add noise to landmarks before input to triangulation
IMPLEMENT_TEST(noisy_landmarks)
{
  using namespace maptk;
  vxl::triangulate_landmarks tri_lm;

  // create landmarks at the corners of a cube
  landmark_map_sptr landmarks = cube_corners(2.0);

  // create a camera sequence (elliptical path)
  camera_map_sptr cameras = camera_seq();

  // create tracks from the projections
  track_set_sptr tracks = projected_tracks(landmarks, cameras);

  // add Gaussian noise to the landmark positions
  landmark_map_sptr landmarks0 = noisy_landmarks(landmarks, 0.1);


  double init_rmse = reprojection_rmse(cameras->cameras(),
                                       landmarks0->landmarks(),
                                       tracks->tracks());
  std::cout << "initial reprojection RMSE: " << init_rmse << std::endl;
  if (init_rmse < 10.0)
  {
    TEST_ERROR("Initial reprojection RMSE should be large before triangulation");
  }

  tri_lm.triangulate(cameras, tracks, landmarks0);

  double end_rmse = reprojection_rmse(cameras->cameras(),
                                      landmarks0->landmarks(),
                                      tracks->tracks());
  TEST_NEAR("RMSE after triangulation", end_rmse, 0.0, 1e-5);
}


// initialize all landmarks to the origin as input to triangulation
IMPLEMENT_TEST(zero_landmarks)
{
  using namespace maptk;
  vxl::triangulate_landmarks tri_lm;

  // create landmarks at the corners of a cube
  landmark_map_sptr landmarks = cube_corners(2.0);

  // create a camera sequence (elliptical path)
  camera_map_sptr cameras = camera_seq();

  // create tracks from the projections
  track_set_sptr tracks = projected_tracks(landmarks, cameras);

  // initialize all landmarks to the origin
  landmark_id_t num_landmarks = static_cast<landmark_id_t>(landmarks->size());
  landmark_map_sptr landmarks0 = init_landmarks(num_landmarks);


  double init_rmse = reprojection_rmse(cameras->cameras(),
                                       landmarks0->landmarks(),
                                       tracks->tracks());
  std::cout << "initial reprojection RMSE: " << init_rmse << std::endl;
  if (init_rmse < 10.0)
  {
    TEST_ERROR("Initial reprojection RMSE should be large before triangulation");
  }

  tri_lm.triangulate(cameras, tracks, landmarks0);

  double end_rmse = reprojection_rmse(cameras->cameras(),
                                      landmarks0->landmarks(),
                                      tracks->tracks());
  TEST_NEAR("RMSE after triangulation", end_rmse, 0.0, 1e-5);
}


// select a subset of cameras to triangulation from
IMPLEMENT_TEST(subset_cameras)
{
  using namespace maptk;
  vxl::triangulate_landmarks tri_lm;

  // create landmarks at the corners of a cube
  landmark_map_sptr landmarks = cube_corners(2.0);

  // create a camera sequence (elliptical path)
  camera_map_sptr cameras = camera_seq();

  // create tracks from the projections
  track_set_sptr tracks = projected_tracks(landmarks, cameras);

  // initialize all landmarks to the origin
  landmark_id_t num_landmarks = static_cast<landmark_id_t>(landmarks->size());
  landmark_map_sptr landmarks0 = init_landmarks(num_landmarks);

  camera_map::map_camera_t cam_map = cameras->cameras();
  camera_map::map_camera_t cam_map2;
  BOOST_FOREACH(camera_map::map_camera_t::value_type& p, cam_map)
  {
    /// take every third camera
    if(p.first % 3 == 0)
    {
      cam_map2.insert(p);
    }
  }
  camera_map_sptr cameras0(new simple_camera_map(cam_map2));


  TEST_EQUAL("Reduced number of cameras", cameras0->size(), 7);

  double init_rmse = reprojection_rmse(cameras0->cameras(),
                                       landmarks0->landmarks(),
                                       tracks->tracks());
  std::cout << "initial reprojection RMSE: " << init_rmse << std::endl;
  if (init_rmse < 10.0)
  {
    TEST_ERROR("Initial reprojection RMSE should be large before triangulation");
  }

  tri_lm.triangulate(cameras0, tracks, landmarks0);

  double end_rmse = reprojection_rmse(cameras0->cameras(),
                                      landmarks0->landmarks(),
                                      tracks->tracks());
  TEST_NEAR("RMSE after triangulation", end_rmse, 0.0, 1e-5);
}


// select a subset of landmarks to triangulate
IMPLEMENT_TEST(subset_landmarks)
{
  using namespace maptk;
  vxl::triangulate_landmarks tri_lm;

  // create landmarks at the corners of a cube
  landmark_map_sptr landmarks = cube_corners(2.0);

  // create a camera sequence (elliptical path)
  camera_map_sptr cameras = camera_seq();

  // create tracks from the projections
  track_set_sptr tracks = projected_tracks(landmarks, cameras);

  // initialize all landmarks to the origin
  landmark_id_t num_landmarks = static_cast<landmark_id_t>(landmarks->size());
  landmark_map_sptr landmarks0 = init_landmarks(num_landmarks);

  // remove some landmarks
  landmark_map::map_landmark_t lm_map = landmarks0->landmarks();
  lm_map.erase(1);
  lm_map.erase(4);
  lm_map.erase(5);
  landmarks0 = landmark_map_sptr(new simple_landmark_map(lm_map));

  TEST_EQUAL("Reduced number of landmarks", landmarks0->size(), 5);

  double init_rmse = reprojection_rmse(cameras->cameras(),
                                       landmarks0->landmarks(),
                                       tracks->tracks());
  std::cout << "initial reprojection RMSE: " << init_rmse << std::endl;
  if (init_rmse < 10.0)
  {
    TEST_ERROR("Initial reprojection RMSE should be large before triangulation");
  }

  tri_lm.triangulate(cameras, tracks, landmarks0);

  double end_rmse = reprojection_rmse(cameras->cameras(),
                                      landmarks0->landmarks(),
                                      tracks->tracks());
  TEST_NEAR("RMSE after triangulation", end_rmse, 0.0, 1e-5);
}


// select a subset of tracks/track_states to constrain the problem
IMPLEMENT_TEST(subset_tracks)
{
  using namespace maptk;
  vxl::triangulate_landmarks tri_lm;

  // create landmarks at the corners of a cube
  landmark_map_sptr landmarks = cube_corners(2.0);

  // create a camera sequence (elliptical path)
  camera_map_sptr cameras = camera_seq();

  // create tracks from the projections
  track_set_sptr tracks = projected_tracks(landmarks, cameras);

  // initialize all landmarks to the origin
  landmark_id_t num_landmarks = static_cast<landmark_id_t>(landmarks->size());
  landmark_map_sptr landmarks0 = init_landmarks(num_landmarks);

  // remove some tracks/track_states
  track_set_sptr tracks0 = subset_tracks(tracks, 0.5);

  double init_rmse = reprojection_rmse(cameras->cameras(),
                                       landmarks0->landmarks(),
                                       tracks0->tracks());
  std::cout << "initial reprojection RMSE: " << init_rmse << std::endl;
  if (init_rmse < 10.0)
  {
    TEST_ERROR("Initial reprojection RMSE should be large before triangulation");
  }

  tri_lm.triangulate(cameras, tracks0, landmarks0);

  double end_rmse = reprojection_rmse(cameras->cameras(),
                                      landmarks0->landmarks(),
                                      tracks0->tracks());
  TEST_NEAR("RMSE after triangulation", end_rmse, 0.0, 1e-5);
}


// select a subset of tracks/track_states and add noise
IMPLEMENT_TEST(noisy_tracks)
{
  using namespace maptk;
  vxl::triangulate_landmarks tri_lm;

  // create landmarks at the corners of a cube
  landmark_map_sptr landmarks = cube_corners(2.0);

  // create a camera sequence (elliptical path)
  camera_map_sptr cameras = camera_seq();

  // create tracks from the projections
  track_set_sptr tracks = projected_tracks(landmarks, cameras);

  // initialize all landmarks to the origin
  landmark_id_t num_landmarks = static_cast<landmark_id_t>(landmarks->size());
  landmark_map_sptr landmarks0 = init_landmarks(num_landmarks);

  // remove some tracks/track_states and add Gaussian noise
  const double track_stdev = 1.0;
  track_set_sptr tracks0 = noisy_tracks(subset_tracks(tracks, 0.5),
                                        track_stdev);


  double init_rmse = reprojection_rmse(cameras->cameras(),
                                       landmarks0->landmarks(),
                                       tracks0->tracks());
  std::cout << "initial reprojection RMSE: " << init_rmse << std::endl;
  if (init_rmse < 10.0)
  {
    TEST_ERROR("Initial reprojection RMSE should be large before triangulation");
  }

  tri_lm.triangulate(cameras, tracks0, landmarks0);

  double end_rmse = reprojection_rmse(cameras->cameras(),
                                      landmarks0->landmarks(),
                                      tracks0->tracks());
  TEST_NEAR("RMSE after triangulation", end_rmse, 0.0, 3.0*track_stdev);
}
