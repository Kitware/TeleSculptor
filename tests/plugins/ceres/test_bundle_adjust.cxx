/*ckwg +29
 * Copyright 2015 by Kitware, Inc.
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
 * \brief test Ceres bundle adjustment functionality
 */

#include <test_common.h>
#include <test_scene.h>

#include <boost/foreach.hpp>

#include <maptk/metrics.h>
#include <maptk/algorithm_plugin_manager.h>
#include <maptk/plugins/ceres/bundle_adjust.h>
#include <maptk/plugins/ceres/register_algorithms.h>
#include <maptk/projected_track_set.h>


#define TEST_ARGS ()

DECLARE_TEST_MAP();

int
main(int argc, char* argv[])
{
  CHECK_ARGS(1);

  // Register ceres algorithm implementations
  maptk::ceres::register_algorithms();

  testname_t const testname = argv[1];

  RUN_TEST(testname);
}


IMPLEMENT_TEST(create)
{
  using namespace maptk;
  algo::bundle_adjust_sptr ba = algo::bundle_adjust::create("ceres");
  if (!ba)
  {
    TEST_ERROR("Unable to create ceres::bundle_adjust by name");
  }
}


// input to SBA is the ideal solution, make sure it doesn't diverge
IMPLEMENT_TEST(from_solution)
{
  using namespace maptk;
  ceres::bundle_adjust ba;
  config_block_sptr cfg = ba.get_configuration();
  cfg->set_value("verbose", "true");
  ba.set_configuration(cfg);

  // create landmarks at the corners of a cube
  landmark_map_sptr landmarks = testing::cube_corners(2.0);

  // create a camera sequence (elliptical path)
  camera_map_sptr cameras = testing::camera_seq();

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

  ba.optimize(cameras, landmarks, tracks);

  double end_rmse = reprojection_rmse(cameras->cameras(),
                                      landmarks->landmarks(),
                                      tracks->tracks());
  TEST_NEAR("RMSE after SBA", end_rmse, 0.0, 1e-12);
}


// add noise to landmarks before input to SBA
IMPLEMENT_TEST(noisy_landmarks)
{
  using namespace maptk;
  ceres::bundle_adjust ba;
  config_block_sptr cfg = ba.get_configuration();
  cfg->set_value("verbose", "true");
  ba.set_configuration(cfg);

  // create landmarks at the corners of a cube
  landmark_map_sptr landmarks = testing::cube_corners(2.0);

  // create a camera sequence (elliptical path)
  camera_map_sptr cameras = testing::camera_seq();

  // create tracks from the projections
  track_set_sptr tracks = projected_tracks(landmarks, cameras);

  // add Gaussian noise to the landmark positions
  landmark_map_sptr landmarks0 = testing::noisy_landmarks(landmarks, 0.1);


  double init_rmse = reprojection_rmse(cameras->cameras(),
                                       landmarks0->landmarks(),
                                       tracks->tracks());
  std::cout << "initial reprojection RMSE: " << init_rmse << std::endl;
  if (init_rmse < 10.0)
  {
    TEST_ERROR("Initial reprojection RMSE should be large before SBA");
  }

  ba.optimize(cameras, landmarks0, tracks);

  double end_rmse = reprojection_rmse(cameras->cameras(),
                                      landmarks0->landmarks(),
                                      tracks->tracks());
  TEST_NEAR("RMSE after SBA", end_rmse, 0.0, 1e-5);
}


// add noise to landmarks and cameras before input to SBA
IMPLEMENT_TEST(noisy_landmarks_noisy_cameras)
{
  using namespace maptk;
  ceres::bundle_adjust ba;
  config_block_sptr cfg = ba.get_configuration();
  cfg->set_value("verbose", "true");
  ba.set_configuration(cfg);

  // create landmarks at the corners of a cube
  landmark_map_sptr landmarks = testing::cube_corners(2.0);

  // create a camera sequence (elliptical path)
  camera_map_sptr cameras = testing::camera_seq();

  // create tracks from the projections
  track_set_sptr tracks = projected_tracks(landmarks, cameras);

  // add Gaussian noise to the landmark positions
  landmark_map_sptr landmarks0 = testing::noisy_landmarks(landmarks, 0.1);

  // add Gaussian noise to the camera positions and orientations
  camera_map_sptr cameras0 = testing::noisy_cameras(cameras, 0.1, 0.1);


  double init_rmse = reprojection_rmse(cameras0->cameras(),
                                       landmarks0->landmarks(),
                                       tracks->tracks());
  std::cout << "initial reprojection RMSE: " << init_rmse << std::endl;
  if (init_rmse < 10.0)
  {
    TEST_ERROR("Initial reprojection RMSE should be large before SBA");
  }

  ba.optimize(cameras0, landmarks0, tracks);

  double end_rmse = reprojection_rmse(cameras0->cameras(),
                                      landmarks0->landmarks(),
                                      tracks->tracks());
  TEST_NEAR("RMSE after SBA", end_rmse, 0.0, 1e-5);
}


// initialize all landmarks to the origin as input to SBA
IMPLEMENT_TEST(zero_landmarks)
{
  using namespace maptk;
  ceres::bundle_adjust ba;
  config_block_sptr cfg = ba.get_configuration();
  cfg->set_value("verbose", "true");
  ba.set_configuration(cfg);

  // create landmarks at the corners of a cube
  landmark_map_sptr landmarks = testing::cube_corners(2.0);

  // create a camera sequence (elliptical path)
  camera_map_sptr cameras = testing::camera_seq();

  // create tracks from the projections
  track_set_sptr tracks = projected_tracks(landmarks, cameras);

  // initialize all landmarks to the origin
  landmark_id_t num_landmarks = static_cast<landmark_id_t>(landmarks->size());
  landmark_map_sptr landmarks0 = testing::init_landmarks(num_landmarks);


  double init_rmse = reprojection_rmse(cameras->cameras(),
                                       landmarks0->landmarks(),
                                       tracks->tracks());
  std::cout << "initial reprojection RMSE: " << init_rmse << std::endl;
  if (init_rmse < 10.0)
  {
    TEST_ERROR("Initial reprojection RMSE should be large before SBA");
  }

  ba.optimize(cameras, landmarks0, tracks);

  double end_rmse = reprojection_rmse(cameras->cameras(),
                                      landmarks0->landmarks(),
                                      tracks->tracks());
  TEST_NEAR("RMSE after SBA", end_rmse, 0.0, 1e-5);
}


// add noise to landmarks and cameras before input to SBA
// select a subset of cameras to optimize
IMPLEMENT_TEST(subset_cameras)
{
  using namespace maptk;
  ceres::bundle_adjust ba;
  config_block_sptr cfg = ba.get_configuration();
  cfg->set_value("verbose", "true");
  ba.set_configuration(cfg);

  // create landmarks at the corners of a cube
  landmark_map_sptr landmarks = testing::cube_corners(2.0);

  // create a camera sequence (elliptical path)
  camera_map_sptr cameras = testing::camera_seq();

  // create tracks from the projections
  track_set_sptr tracks = projected_tracks(landmarks, cameras);

  // add Gaussian noise to the landmark positions
  landmark_map_sptr landmarks0 = testing::noisy_landmarks(landmarks, 0.1);

  // add Gaussian noise to the camera positions and orientations
  camera_map_sptr cameras0 = testing::noisy_cameras(cameras, 0.1, 0.1);

  camera_map::map_camera_t cam_map = cameras0->cameras();
  camera_map::map_camera_t cam_map2;
  BOOST_FOREACH(camera_map::map_camera_t::value_type& p, cam_map)
  {
    /// take every third camera
    if(p.first % 3 == 0)
    {
      cam_map2.insert(p);
    }
  }
  cameras0 = camera_map_sptr(new simple_camera_map(cam_map2));


  TEST_EQUAL("Reduced number of cameras", cameras0->size(), 7);

  double init_rmse = reprojection_rmse(cameras0->cameras(),
                                       landmarks0->landmarks(),
                                       tracks->tracks());
  std::cout << "initial reprojection RMSE: " << init_rmse << std::endl;
  if (init_rmse < 10.0)
  {
    TEST_ERROR("Initial reprojection RMSE should be large before SBA");
  }

  ba.optimize(cameras0, landmarks0, tracks);

  double end_rmse = reprojection_rmse(cameras0->cameras(),
                                      landmarks0->landmarks(),
                                      tracks->tracks());
  TEST_NEAR("RMSE after SBA", end_rmse, 0.0, 1e-5);
}


// add noise to landmarks and cameras before input to SBA
// select a subset of landmarks to optimize
IMPLEMENT_TEST(subset_landmarks)
{
  using namespace maptk;
  ceres::bundle_adjust ba;
  config_block_sptr cfg = ba.get_configuration();
  cfg->set_value("verbose", "true");
  ba.set_configuration(cfg);

  // create landmarks at the corners of a cube
  landmark_map_sptr landmarks = testing::cube_corners(2.0);

  // create a camera sequence (elliptical path)
  camera_map_sptr cameras = testing::camera_seq();

  // create tracks from the projections
  track_set_sptr tracks = projected_tracks(landmarks, cameras);

  // add Gaussian noise to the landmark positions
  landmark_map_sptr landmarks0 = testing::noisy_landmarks(landmarks, 0.1);

  // add Gaussian noise to the camera positions and orientations
  camera_map_sptr cameras0 = testing::noisy_cameras(cameras, 0.1, 0.1);

  // remove some landmarks
  landmark_map::map_landmark_t lm_map = landmarks0->landmarks();
  lm_map.erase(1);
  lm_map.erase(4);
  lm_map.erase(5);
  landmarks0 = landmark_map_sptr(new simple_landmark_map(lm_map));

  TEST_EQUAL("Reduced number of landmarks", landmarks0->size(), 5);

  double init_rmse = reprojection_rmse(cameras0->cameras(),
                                       landmarks0->landmarks(),
                                       tracks->tracks());
  std::cout << "initial reprojection RMSE: " << init_rmse << std::endl;
  if (init_rmse < 10.0)
  {
    TEST_ERROR("Initial reprojection RMSE should be large before SBA");
  }

  ba.optimize(cameras0, landmarks0, tracks);

  double end_rmse = reprojection_rmse(cameras0->cameras(),
                                      landmarks0->landmarks(),
                                      tracks->tracks());
  TEST_NEAR("RMSE after SBA", end_rmse, 0.0, 1e-5);
}


// add noise to landmarks and cameras before input to SBA
// select a subset of tracks/track_states to constrain the problem
IMPLEMENT_TEST(subset_tracks)
{
  using namespace maptk;
  ceres::bundle_adjust ba;
  config_block_sptr cfg = ba.get_configuration();
  cfg->set_value("verbose", "true");
  ba.set_configuration(cfg);

  // create landmarks at the corners of a cube
  landmark_map_sptr landmarks = testing::cube_corners(2.0);

  // create a camera sequence (elliptical path)
  camera_map_sptr cameras = testing::camera_seq();

  // create tracks from the projections
  track_set_sptr tracks = projected_tracks(landmarks, cameras);

  // add Gaussian noise to the landmark positions
  landmark_map_sptr landmarks0 = testing::noisy_landmarks(landmarks, 0.1);

  // add Gaussian noise to the camera positions and orientations
  camera_map_sptr cameras0 = testing::noisy_cameras(cameras, 0.1, 0.1);

  // remove some tracks/track_states
  track_set_sptr tracks0 = testing::subset_tracks(tracks, 0.5);


  double init_rmse = reprojection_rmse(cameras0->cameras(),
                                       landmarks0->landmarks(),
                                       tracks0->tracks());
  std::cout << "initial reprojection RMSE: " << init_rmse << std::endl;
  if (init_rmse < 10.0)
  {
    TEST_ERROR("Initial reprojection RMSE should be large before SBA");
  }

  ba.optimize(cameras0, landmarks0, tracks0);

  double end_rmse = reprojection_rmse(cameras0->cameras(),
                                      landmarks0->landmarks(),
                                      tracks0->tracks());
  TEST_NEAR("RMSE after SBA", end_rmse, 0.0, 1e-5);
}


// add noise to landmarks and cameras and tracks before input to SBA
// select a subset of tracks/track_states and add observation noise
IMPLEMENT_TEST(noisy_tracks)
{
  using namespace maptk;
  ceres::bundle_adjust ba;
  config_block_sptr cfg = ba.get_configuration();
  cfg->set_value("verbose", "true");
  ba.set_configuration(cfg);

  // create landmarks at the corners of a cube
  landmark_map_sptr landmarks = testing::cube_corners(2.0);

  // create a camera sequence (elliptical path)
  camera_map_sptr cameras = testing::camera_seq();

  // create tracks from the projections
  track_set_sptr tracks = projected_tracks(landmarks, cameras);

  // add Gaussian noise to the landmark positions
  landmark_map_sptr landmarks0 = testing::noisy_landmarks(landmarks, 0.1);

  // add Gaussian noise to the camera positions and orientations
  camera_map_sptr cameras0 = testing::noisy_cameras(cameras, 0.1, 0.1);

  // remove some tracks/track_states and add Gaussian noise
  const double track_stdev = 1.0;
  track_set_sptr tracks0 = testing::noisy_tracks(
                               testing::subset_tracks(tracks, 0.5),
                               track_stdev);


  double init_rmse = reprojection_rmse(cameras0->cameras(),
                                       landmarks0->landmarks(),
                                       tracks0->tracks());
  std::cout << "initial reprojection RMSE: " << init_rmse << std::endl;
  if (init_rmse < 10.0)
  {
    TEST_ERROR("Initial reprojection RMSE should be large before SBA");
  }

  ba.optimize(cameras0, landmarks0, tracks0);

  double end_rmse = reprojection_rmse(cameras0->cameras(),
                                      landmarks0->landmarks(),
                                      tracks0->tracks());
  TEST_NEAR("RMSE after SBA", end_rmse, 0.0, track_stdev);
}


// add noise to landmarks and cameras and tracks before input to SBA
// select a subset of tracks_states to make outliers (large observation noise)
// add a small amount of noise to all track states
// select a subset of tracks/track_states to constrain the problem
IMPLEMENT_TEST(outlier_tracks)
{
  using namespace maptk;
  ceres::bundle_adjust ba;
  config_block_sptr cfg = ba.get_configuration();
  cfg->set_value("verbose", "true");
  cfg->set_value("max_num_iterations", 100);
  ba.set_configuration(cfg);

  // create landmarks at the corners of a cube
  landmark_map_sptr landmarks = testing::cube_corners(2.0);

  // create a camera sequence (elliptical path)
  camera_map_sptr cameras = testing::camera_seq();

  // create tracks from the projections
  track_set_sptr tracks = projected_tracks(landmarks, cameras);

  // add Gaussian noise to the landmark positions
  landmark_map_sptr landmarks0 = testing::noisy_landmarks(landmarks, 0.1);

  // add Gaussian noise to the camera positions and orientations
  camera_map_sptr cameras0 = testing::noisy_cameras(cameras, 0.1, 0.1);

  // make some observations outliers
  track_set_sptr tracks_w_outliers =
      testing::add_outliers_to_tracks(tracks, 0.1, 20.0);

  // remove some tracks/track_states and add Gaussian noise
  const double track_stdev = 1.0;
  track_set_sptr tracks0 = testing::noisy_tracks(
                               testing::subset_tracks(tracks_w_outliers, 0.5),
                               track_stdev);


  double init_rmse = reprojection_rmse(cameras0->cameras(),
                                       landmarks0->landmarks(),
                                       tracks0->tracks());
  std::cout << "initial reprojection RMSE: " << init_rmse << std::endl;
  if (init_rmse < 10.0)
  {
    TEST_ERROR("Initial reprojection RMSE should be large before SBA");
  }

  double init_med_err = reprojection_median_error(cameras0->cameras(),
                                                  landmarks0->landmarks(),
                                                  tracks0->tracks());
  std::cout << "initial reprojection median error: "
            << init_med_err << std::endl;
  if (init_med_err < 10.0)
  {
    TEST_ERROR("Initial reprojection median_error should be large before SBA");
  }

  // make a copy of the initial cameras and landmarks
  landmark_map_sptr landmarks1(new simple_landmark_map(landmarks0->landmarks()));
  camera_map_sptr cameras1(new simple_camera_map(cameras0->cameras()));

  // run bundle adjustement with the default, non-robust, trivial loss function
  ba.optimize(cameras0, landmarks0, tracks0);

  double trivial_loss_rmse = reprojection_rmse(cameras0->cameras(),
                                               landmarks0->landmarks(),
                                               tracks0->tracks());
  double trivial_loss_med_err = reprojection_median_error(cameras0->cameras(),
                                                          landmarks0->landmarks(),
                                                          tracks0->tracks());

  std::cout << "Non-robust SBA mean/median reprojection error: "
            << trivial_loss_rmse << "/" << trivial_loss_med_err << std::endl;
  if ( trivial_loss_med_err < track_stdev )
  {
    TEST_ERROR("Non-robust SBA should have a large median residual");
  }

  // run bundle adjustment with a robust loss function
  cfg->set_value("loss_function_type", "HUBER_LOSS");
  ba.set_configuration(cfg);
  ba.optimize(cameras1, landmarks1, tracks0);

  double robust_loss_rmse = reprojection_rmse(cameras1->cameras(),
                                               landmarks1->landmarks(),
                                               tracks0->tracks());
  double robust_loss_med_err = reprojection_median_error(cameras1->cameras(),
                                                         landmarks1->landmarks(),
                                                         tracks0->tracks());

  std::cout << "Robust SBA mean/median reprojection error: "
            << robust_loss_rmse << "/" << robust_loss_med_err << std::endl;
  if ( trivial_loss_rmse > robust_loss_rmse )
  {
    TEST_ERROR("Robust SBA should increase RMSE error");
  }
  if ( trivial_loss_med_err <= robust_loss_med_err )
  {
    TEST_ERROR("Robust SBA should decrease median error");
  }
  TEST_NEAR("median error after robust SBA", robust_loss_med_err, 0.0, track_stdev);

}


// helper for tests using distortion models in bundle adjustment
void test_ba_using_distortion(maptk::config_block_sptr cfg,
                              const Eigen::VectorXd& dc,
                              bool clear_init_distortion)
{
  using namespace maptk;
  ceres::bundle_adjust ba;
  cfg->set_value("verbose", "true");
  ba.set_configuration(cfg);

  // create landmarks at the corners of a cube
  landmark_map_sptr landmarks = testing::cube_corners(2.0);

  // The intrinsic camera parameters to use
  camera_intrinsics_d K(1000, vector_2d(640,480));
  K.set_dist_coeffs(dc);

  // create a camera sequence (elliptical path)
  camera_map_sptr cameras = testing::camera_seq(20,K);

  // create tracks from the projections
  track_set_sptr tracks = projected_tracks(landmarks, cameras);

  // add Gaussian noise to the landmark positions
  landmark_map_sptr landmarks0 = testing::noisy_landmarks(landmarks, 0.1);

  if( clear_init_distortion )
  {
    // regenerate cameras without distortion so we can try to recover it
    K.set_dist_coeffs(Eigen::VectorXd());
    cameras = testing::camera_seq(20,K);
  }

  // add Gaussian noise to the camera positions and orientations
  camera_map_sptr cameras0 = testing::noisy_cameras(cameras, 0.1, 0.1);

  double init_rmse = reprojection_rmse(cameras0->cameras(),
                                       landmarks0->landmarks(),
                                       tracks->tracks());
  std::cout << "initial reprojection RMSE: " << init_rmse << std::endl;
  if (init_rmse < 10.0)
  {
    TEST_ERROR("Initial reprojection RMSE should be large before SBA");
  }

  ba.optimize(cameras0, landmarks0, tracks);

  double end_rmse = reprojection_rmse(cameras0->cameras(),
                                      landmarks0->landmarks(),
                                      tracks->tracks());
  TEST_NEAR("RMSE after SBA", end_rmse, 0.0, 1e-7);

  // compare actual to estimated distortion parameters
  if( clear_init_distortion )
  {
    Eigen::VectorXd dc2 = cameras0->cameras()[0]->intrinsics().dist_coeffs();
    // The estimated parameter vector can be longer and zero padded
    if( dc2.size() > dc.size() )
    {
      dc2 = dc2.head(dc.size());
    }
    Eigen::VectorXd diff = (dc2-dc).cwiseAbs();
    std::cout << "distortion parameters\n"
              << "  actual:   " << dc.transpose() << "\n"
              << "  estimated: " << dc2.transpose() << "\n"
              << "  difference: " << diff.transpose() << std::endl;
  }
}


// use 1 lens distortion coefficent in bundle adjustment
IMPLEMENT_TEST(use_lens_distortion_1)
{
  using namespace maptk;
  ceres::bundle_adjust ba;
  config_block_sptr cfg = ba.get_configuration();
  cfg->set_value("lens_distortion_type", "POLYNOMIAL_RADIAL_DISTORTION");
  cfg->set_value("optimize_dist_k1", false);
  cfg->set_value("optimize_dist_k2", false);

  // The lens distortion parameters to use
  Eigen::VectorXd dc(1);
  dc << -0.01;

  test_ba_using_distortion(cfg, dc, false);
}


// use 2 lens distortion coefficents in bundle adjustment
IMPLEMENT_TEST(use_lens_distortion_2)
{
  using namespace maptk;
  ceres::bundle_adjust ba;
  config_block_sptr cfg = ba.get_configuration();
  cfg->set_value("lens_distortion_type", "POLYNOMIAL_RADIAL_DISTORTION");
  cfg->set_value("optimize_dist_k1", false);
  cfg->set_value("optimize_dist_k2", false);

  // The lens distortion parameters to use
  Eigen::VectorXd dc(2);
  dc << -0.01, 0.002;

  test_ba_using_distortion(cfg, dc, false);
}


// use 3 lens distortion coefficents in bundle adjustment
IMPLEMENT_TEST(use_lens_distortion_3)
{
  using namespace maptk;
  ceres::bundle_adjust ba;
  config_block_sptr cfg = ba.get_configuration();
  cfg->set_value("lens_distortion_type",
                 "POLYNOMIAL_RADIAL_TANGENTIAL_DISTORTION");
  cfg->set_value("optimize_dist_k1", false);
  cfg->set_value("optimize_dist_k2", false);
  cfg->set_value("optimize_dist_k3", false);
  cfg->set_value("optimize_dist_p1_p2", false);

  // The lens distortion parameters to use
  Eigen::VectorXd dc(5);
  dc << -0.01, 0.002, 0, 0, -0.005;

  test_ba_using_distortion(cfg, dc, false);
}


// use 5 lens distortion coefficents in bundle adjustment
IMPLEMENT_TEST(use_lens_distortion_5)
{
  using namespace maptk;
  ceres::bundle_adjust ba;
  config_block_sptr cfg = ba.get_configuration();
  cfg->set_value("lens_distortion_type",
                 "POLYNOMIAL_RADIAL_TANGENTIAL_DISTORTION");
  cfg->set_value("optimize_dist_k1", false);
  cfg->set_value("optimize_dist_k2", false);
  cfg->set_value("optimize_dist_k3", false);
  cfg->set_value("optimize_dist_p1_p2", false);

  // The lens distortion parameters to use
  Eigen::VectorXd dc(5);
  dc << -0.01, 0.002, -0.0005, 0.001, -0.005;

  test_ba_using_distortion(cfg, dc, false);
}


// use 8 lens distortion coefficents in bundle adjustment
IMPLEMENT_TEST(use_lens_distortion_8)
{
  using namespace maptk;
  ceres::bundle_adjust ba;
  config_block_sptr cfg = ba.get_configuration();
  cfg->set_value("lens_distortion_type",
                 "RATIONAL_RADIAL_TANGENTIAL_DISTORTION");
  cfg->set_value("optimize_dist_k1", false);
  cfg->set_value("optimize_dist_k2", false);
  cfg->set_value("optimize_dist_k3", false);
  cfg->set_value("optimize_dist_p1_p2", false);
  cfg->set_value("optimize_dist_k4_k5_k6", false);

  // The lens distortion parameters to use
  Eigen::VectorXd dc(8);
  dc << -0.01, 0.002, -0.0005, 0.001, -0.005, 0.02, 0.0007, -0.003;

  test_ba_using_distortion(cfg, dc, false);
}


// estimate 1 lens distortion coefficent in bundle adjustment
IMPLEMENT_TEST(est_lens_distortion_1)
{
  using namespace maptk;
  ceres::bundle_adjust ba;
  config_block_sptr cfg = ba.get_configuration();
  cfg->set_value("lens_distortion_type", "POLYNOMIAL_RADIAL_DISTORTION");
  cfg->set_value("optimize_dist_k1", true);
  cfg->set_value("optimize_dist_k2", false);

  // The lens distortion parameters to use
  Eigen::VectorXd dc(1);
  dc << -0.01;

  test_ba_using_distortion(cfg, dc, true);
}


// estimate 2 lens distortion coefficents in bundle adjustment
IMPLEMENT_TEST(est_lens_distortion_2)
{
  using namespace maptk;
  ceres::bundle_adjust ba;
  config_block_sptr cfg = ba.get_configuration();
  cfg->set_value("lens_distortion_type", "POLYNOMIAL_RADIAL_DISTORTION");
  cfg->set_value("optimize_dist_k1", true);
  cfg->set_value("optimize_dist_k2", true);

  // The lens distortion parameters to use
  Eigen::VectorXd dc(2);
  dc << -0.01, 0.002;

  test_ba_using_distortion(cfg, dc, true);
}


// estimate 3 lens distortion coefficents in bundle adjustment
IMPLEMENT_TEST(est_lens_distortion_3)
{
  using namespace maptk;
  ceres::bundle_adjust ba;
  config_block_sptr cfg = ba.get_configuration();
  cfg->set_value("lens_distortion_type",
                 "POLYNOMIAL_RADIAL_TANGENTIAL_DISTORTION");
  cfg->set_value("optimize_dist_k1", true);
  cfg->set_value("optimize_dist_k2", true);
  cfg->set_value("optimize_dist_k3", true);
  cfg->set_value("optimize_dist_p1_p2", false);

  // The lens distortion parameters to use
  Eigen::VectorXd dc(5);
  dc << -0.01, 0.002, 0, 0, -0.005;

  test_ba_using_distortion(cfg, dc, true);
}


// estimate 5 lens distortion coefficents in bundle adjustment
IMPLEMENT_TEST(est_lens_distortion_5)
{
  using namespace maptk;
  ceres::bundle_adjust ba;
  config_block_sptr cfg = ba.get_configuration();
  cfg->set_value("lens_distortion_type",
                 "POLYNOMIAL_RADIAL_TANGENTIAL_DISTORTION");
  cfg->set_value("optimize_dist_k1", true);
  cfg->set_value("optimize_dist_k2", true);
  cfg->set_value("optimize_dist_k3", true);
  cfg->set_value("optimize_dist_p1_p2", true);

  // The lens distortion parameters to use
  Eigen::VectorXd dc(5);
  dc << -0.01, 0.002, -0.0005, 0.001, -0.005;

  test_ba_using_distortion(cfg, dc, true);
}


// estimate 8 lens distortion coefficents in bundle adjustment
IMPLEMENT_TEST(est_lens_distortion_8)
{
  using namespace maptk;
  ceres::bundle_adjust ba;
  config_block_sptr cfg = ba.get_configuration();
  cfg->set_value("lens_distortion_type",
                 "RATIONAL_RADIAL_TANGENTIAL_DISTORTION");
  cfg->set_value("optimize_dist_k1", true);
  cfg->set_value("optimize_dist_k2", true);
  cfg->set_value("optimize_dist_k3", true);
  cfg->set_value("optimize_dist_p1_p2", true);
  cfg->set_value("optimize_dist_k4_k5_k6", true);

  // The lens distortion parameters to use
  Eigen::VectorXd dc(8);
  dc << -0.01, 0.002, -0.0005, 0.001, -0.005, 0.02, 0.0007, -0.003;

  test_ba_using_distortion(cfg, dc, true);
}
