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
 * \brief test Ceres reprojection error functors
 */

#include <test_common.h>
#include <test_scene.h>

#include <vital/vital_foreach.h>

#include <maptk/metrics.h>
#include <maptk/plugins/ceres/reprojection_error.h>
#include <maptk/plugins/ceres/types.h>
#include <maptk/projected_track_set.h>


#define TEST_ARGS ()

DECLARE_TEST_MAP();

int
main(int argc, char* argv[])
{
  CHECK_ARGS(1);

  testname_t const testname = argv[1];

  RUN_TEST(testname);
}


/// test the reprojection error of a single residual
void
test_reprojection_error(const kwiver::vital::camera& cam,
                        const kwiver::vital::landmark& lm,
                        const kwiver::vital::feature& f,
                        const kwiver::maptk::ceres::LensDistortionType dist_type)
{
  using namespace kwiver::maptk;
  using namespace kwiver::maptk::ceres;
  using namespace kwiver::vital;

  ::ceres::CostFunction* cost_func =
      create_cost_func(dist_type, f.loc().x(), f.loc().y());

  double pose[6];
  vector_3d rot = cam.rotation().rodrigues();
  std::copy(rot.data(), rot.data()+3, pose);
  vector_3d center = cam.center();
  std::copy(center.data(), center.data()+3, pose+3);

  unsigned int ndp = num_distortion_params(dist_type);
  std::vector<double> intrinsics(5+ndp, 0.0);
  camera_intrinsics_d K = cam.intrinsics();
  intrinsics[0] = K.focal_length();
  intrinsics[1] = K.principal_point().x();
  intrinsics[2] = K.principal_point().y();
  intrinsics[3] = K.aspect_ratio();
  intrinsics[4] = K.skew();
  const Eigen::VectorXd& d = K.dist_coeffs();
  // copy the intersection of parameters provided in K
  // and those that are supported by the requested model type
  unsigned int num_dp = std::min(ndp, static_cast<unsigned int>(d.size()));
  std::copy(d.data(), d.data()+num_dp, &intrinsics[5]);

  double point[3] = {lm.loc().x(), lm.loc().y(), lm.loc().z()};

  double* parameters[3] = {&intrinsics[0], pose, point};
  vector_2d residuals;
  cost_func->Evaluate(parameters, residuals.data(), NULL);
  delete cost_func;

  TEST_NEAR("Residual near zero", residuals.norm(), 0.0, 1e-12);
}


/// test the reprojection error of all residuals
void
test_all_reprojection_errors(const kwiver::vital::camera_map_sptr cameras,
                             const kwiver::vital::landmark_map_sptr landmarks,
                             const kwiver::vital::track_set_sptr tracks,
                             const kwiver::maptk::ceres::LensDistortionType dist_type)
{
  using namespace kwiver::maptk;
  using namespace kwiver::vital;

  camera_map::map_camera_t cam_map = cameras->cameras();
  landmark_map::map_landmark_t lm_map = landmarks->landmarks();
  std::vector<track_sptr> trks = tracks->tracks();

  double rmse = reprojection_rmse(cam_map, lm_map, trks);
  std::cout << "MAP-Tk reprojection RMSE: " << rmse << std::endl;
  TEST_NEAR("MAP-Tk reprojection RMSE should be small", rmse, 0.0, 1e-12);

  typedef std::map<landmark_id_t, landmark_sptr>::const_iterator lm_map_itr_t;
  typedef std::map<frame_id_t, camera_sptr>::const_iterator cam_map_itr_t;
  VITAL_FOREACH(const track_sptr& t, trks)
  {
    lm_map_itr_t lmi = lm_map.find(t->id());
    if (lmi == lm_map.end() || !lmi->second)
    {
      // no landmark corresponding to this track
      continue;
    }
    const landmark& lm = *lmi->second;
    for( track::history_const_itr tsi = t->begin(); tsi != t->end(); ++tsi)
    {
      if (!tsi->feat)
      {
        // no feature for this track state.
        continue;
      }
      const feature& feat = *tsi->feat;
      cam_map_itr_t ci = cam_map.find(tsi->frame_id);
      if (ci == cam_map.end() || !ci->second)
      {
        // no camera corresponding to this track state
        continue;
      }
      const camera& cam = *ci->second;
      test_reprojection_error(cam, lm, feat, dist_type);
    }
  }
}


/// test reprojection error on a test scene for a particular model
void test_reprojection_model(const Eigen::VectorXd& dc,
                             const kwiver::maptk::ceres::LensDistortionType dist_type)
{
  using namespace kwiver::maptk;
  using namespace kwiver::maptk::ceres;
  using namespace kwiver::vital;

  // create landmarks at the corners of a cube
  landmark_map_sptr landmarks = testing::cube_corners(2.0);

  // The intrinsic camera parameters to use
  camera_intrinsics_d K(1000, vector_2d(640,480));
  K.set_dist_coeffs(dc);

  // create a camera sequence (elliptical path)
  camera_map_sptr cameras = testing::camera_seq(20,K);

  // create tracks from the projections
  track_set_sptr tracks = projected_tracks(landmarks, cameras);

  test_all_reprojection_errors(cameras, landmarks, tracks,
                               dist_type);
}


// compare MAP-Tk camera projection to Ceres reprojection error models
IMPLEMENT_TEST(compare_projections_no_distortion)
{
  using namespace kwiver::maptk::ceres;
  using namespace kwiver::vital;

  Eigen::VectorXd dc;

  std::cout << "Testing NO_DISTORTION model" << std::endl;
  test_reprojection_model(dc, NO_DISTORTION);
  std::cout << "Testing POLYNOMIAL_RADIAL_DISTORTION model" << std::endl;
  test_reprojection_model(dc, POLYNOMIAL_RADIAL_DISTORTION);
  std::cout << "Testing POLYNOMIAL_RADIAL_TANGENTIAL_DISTORTION model" << std::endl;
  test_reprojection_model(dc, POLYNOMIAL_RADIAL_TANGENTIAL_DISTORTION);
  std::cout << "Testing RATIONAL_RADIAL_TANGENTIAL_DISTORTION model" << std::endl;
  test_reprojection_model(dc, RATIONAL_RADIAL_TANGENTIAL_DISTORTION);
}


// compare MAP-Tk camera projection to Ceres reprojection error models
IMPLEMENT_TEST(compare_projections_distortion_1)
{
  using namespace kwiver::maptk::ceres;

  Eigen::VectorXd dc(1);
  dc << -0.01;

  std::cout << "Testing POLYNOMIAL_RADIAL_DISTORTION model" << std::endl;
  test_reprojection_model(dc, POLYNOMIAL_RADIAL_DISTORTION);
  std::cout << "Testing POLYNOMIAL_RADIAL_TANGENTIAL_DISTORTION model" << std::endl;
  test_reprojection_model(dc, POLYNOMIAL_RADIAL_TANGENTIAL_DISTORTION);
  std::cout << "Testing RATIONAL_RADIAL_TANGENTIAL_DISTORTION model" << std::endl;
  test_reprojection_model(dc, RATIONAL_RADIAL_TANGENTIAL_DISTORTION);
}


// compare MAP-Tk camera projection to Ceres reprojection error models
IMPLEMENT_TEST(compare_projections_distortion_2)
{
  using namespace kwiver::maptk::ceres;

  Eigen::VectorXd dc(2);
  dc << -0.01, 0.002;

  std::cout << "Testing POLYNOMIAL_RADIAL_DISTORTION model" << std::endl;
  test_reprojection_model(dc, POLYNOMIAL_RADIAL_DISTORTION);
  std::cout << "Testing POLYNOMIAL_RADIAL_TANGENTIAL_DISTORTION model" << std::endl;
  test_reprojection_model(dc, POLYNOMIAL_RADIAL_TANGENTIAL_DISTORTION);
  std::cout << "Testing RATIONAL_RADIAL_TANGENTIAL_DISTORTION model" << std::endl;
  test_reprojection_model(dc, RATIONAL_RADIAL_TANGENTIAL_DISTORTION);
}


// compare MAP-Tk camera projection to Ceres reprojection error models
IMPLEMENT_TEST(compare_projections_distortion_4)
{
  using namespace kwiver::maptk::ceres;

  Eigen::VectorXd dc(4);
  dc << -0.01, 0.002, 0.001, -0.005;

  std::cout << "Testing POLYNOMIAL_RADIAL_TANGENTIAL_DISTORTION model" << std::endl;
  test_reprojection_model(dc, POLYNOMIAL_RADIAL_TANGENTIAL_DISTORTION);
  std::cout << "Testing RATIONAL_RADIAL_TANGENTIAL_DISTORTION model" << std::endl;
  test_reprojection_model(dc, RATIONAL_RADIAL_TANGENTIAL_DISTORTION);
}


// compare MAP-Tk camera projection to Ceres reprojection error models
IMPLEMENT_TEST(compare_projections_distortion_5)
{
  using namespace kwiver::maptk::ceres;

  Eigen::VectorXd dc(5);
  dc << -0.01, 0.002, 0.001, -0.005, -0.004;

  std::cout << "Testing POLYNOMIAL_RADIAL_TANGENTIAL_DISTORTION model" << std::endl;
  test_reprojection_model(dc, POLYNOMIAL_RADIAL_TANGENTIAL_DISTORTION);
  std::cout << "Testing RATIONAL_RADIAL_TANGENTIAL_DISTORTION model" << std::endl;
  test_reprojection_model(dc, RATIONAL_RADIAL_TANGENTIAL_DISTORTION);
}


// compare MAP-Tk camera projection to Ceres reprojection error models
IMPLEMENT_TEST(compare_projections_distortion_8)
{
  using namespace kwiver::maptk::ceres;

  Eigen::VectorXd dc(8);
  dc << -0.01, 0.002, 0.001, -0.005, -0.004, 0.02, -0.007, 0.0001;

  std::cout << "Testing RATIONAL_RADIAL_TANGENTIAL_DISTORTION model" << std::endl;
  test_reprojection_model(dc, RATIONAL_RADIAL_TANGENTIAL_DISTORTION);
}
