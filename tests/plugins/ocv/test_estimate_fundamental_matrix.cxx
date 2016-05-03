/*ckwg +29
 * Copyright 2016 by Kitware, Inc.
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
#include <test_random_point.h>

#include <maptk/projected_track_set.h>
#include <maptk/metrics.h>
#include <maptk/epipolar_geometry.h>
#include <maptk/plugins/ocv/estimate_fundamental_matrix.h>
#include <maptk/plugins/ocv/register_algorithms.h>

#include <Eigen/LU>


#define TEST_ARGS ()

DECLARE_TEST_MAP();

int
main(int argc, char* argv[])
{
  CHECK_ARGS(1);

  kwiver::maptk::ocv::register_algorithms();

  testname_t const testname = argv[1];

  RUN_TEST(testname);
}

using namespace kwiver::vital;

IMPLEMENT_TEST(create)
{
  using namespace kwiver::maptk;
  algo::estimate_fundamental_matrix_sptr est_e = algo::estimate_fundamental_matrix::create("ocv");
  if (!est_e)
  {
    TEST_ERROR("Unable to create ocv::estimate_fundamental_matrix by name");
  }
}


// Print epipolar distance of pairs of points given a fundamental matrix
void print_epipolar_distances(const kwiver::vital::matrix_3x3d& F,
                              const std::vector<kwiver::vital::vector_2d> right_pts,
                              const std::vector<kwiver::vital::vector_2d> left_pts)
{
  using namespace kwiver::maptk;
  matrix_3x3d Ft = F.transpose();
  for(unsigned i=0; i<right_pts.size(); ++i)
  {
    const vector_2d& pr = right_pts[i];
    const vector_2d& pl = left_pts[i];
    vector_3d vr(pr.x(), pr.y(), 1.0);
    vector_3d vl(pl.x(), pl.y(), 1.0);
    vector_3d lr = F * vr;
    vector_3d ll = Ft * vl;
    double sr = 1.0 / sqrt(lr.x()*lr.x() + lr.y()*lr.y());
    double sl = 1.0 / sqrt(ll.x()*ll.x() + ll.y()*ll.y());
    // sum of point to epipolar line distance in both images
    double d = vr.dot(ll);
    std::cout <<" dist right = "<<d*sr<<"  dist left = "<<d*sl << std::endl;
  }
}


// test fundamental matrix estimation with ideal points
IMPLEMENT_TEST(ideal_points)
{
  using namespace kwiver::maptk;
  ocv::estimate_fundamental_matrix est_f;

  // create landmarks at the random locations
  landmark_map_sptr landmarks = testing::init_landmarks(100);
  landmarks = testing::noisy_landmarks(landmarks, 1.0);

  // create a camera sequence (elliptical path)
  camera_map_sptr cameras = testing::camera_seq();

  // create tracks from the projections
  track_set_sptr tracks = projected_tracks(landmarks, cameras);

  const frame_id_t frame1 = 0;
  const frame_id_t frame2 = 10;

  camera_map::map_camera_t cams = cameras->cameras();
  camera_sptr cam1 = cams[frame1];
  camera_sptr cam2 = cams[frame2];
  camera_intrinsics_sptr cal1 = cam1->intrinsics();
  camera_intrinsics_sptr cal2 = cam2->intrinsics();

  // compute the true fundamental matrix from the cameras
  fundamental_matrix_sptr true_F = fundamental_matrix_from_cameras(*cam1, *cam2);

  // extract coresponding image points
  std::vector<track_sptr> trks = tracks->tracks();
  std::vector<vector_2d> pts1, pts2;
  for(unsigned int i=0; i<trks.size(); ++i)
  {
    pts1.push_back(trks[i]->find(frame1)->feat->loc());
    pts2.push_back(trks[i]->find(frame2)->feat->loc());
  }

  // print the epipolar distances using this fundamental matrix
  print_epipolar_distances(true_F->matrix(), pts1, pts2);

  // compute the fundmental matrix from the corresponding points
  std::vector<bool> inliers;
  fundamental_matrix_sptr F_sptr = est_f.estimate(pts1, pts2,
                                                  inliers, 1.5);
  matrix_3x3d F = F_sptr->matrix();
  // check for sign difference
  if( true_F->matrix().cwiseProduct(F).sum() < 0.0 )
  {
    F *= -1;
  }

  // compare true and computed fundamental matrices
  std::cout << "true F = "<<*true_F<<std::endl;
  std::cout << "Estimated F = "<< F <<std::endl;
  TEST_NEAR("Fundamental Matrix Estimate", F, true_F->matrix(), 1e-6);

  unsigned num_inliers = static_cast<unsigned>(std::count(inliers.begin(),
                                                          inliers.end(), true));
  std::cout << "num inliers "<<num_inliers<<std::endl;
  TEST_EQUAL("All points are inliers", num_inliers, pts1.size());
}


// test fundamental matrix estimation with noisy points
IMPLEMENT_TEST(noisy_points)
{
  using namespace kwiver::maptk;
  ocv::estimate_fundamental_matrix est_f;

  // create landmarks at the random locations
  landmark_map_sptr landmarks = testing::init_landmarks(100);
  landmarks = testing::noisy_landmarks(landmarks, 1.0);

  // create a camera sequence (elliptical path)
  camera_map_sptr cameras = testing::camera_seq();

  // create tracks from the projections
  track_set_sptr tracks = projected_tracks(landmarks, cameras);

  // add random noise to track image locations
  tracks = testing::noisy_tracks(tracks, 0.5);

  const frame_id_t frame1 = 0;
  const frame_id_t frame2 = 10;

  camera_map::map_camera_t cams = cameras->cameras();
  camera_sptr cam1 = cams[frame1];
  camera_sptr cam2 = cams[frame2];
  camera_intrinsics_sptr cal1 = cam1->intrinsics();
  camera_intrinsics_sptr cal2 = cam2->intrinsics();

  // compute the true fundamental matrix from the cameras
  fundamental_matrix_sptr true_F = fundamental_matrix_from_cameras(*cam1, *cam2);

  // extract coresponding image points
  std::vector<track_sptr> trks = tracks->tracks();
  std::vector<vector_2d> pts1, pts2;
  for(unsigned int i=0; i<trks.size(); ++i)
  {
    pts1.push_back(trks[i]->find(frame1)->feat->loc());
    pts2.push_back(trks[i]->find(frame2)->feat->loc());
  }

  print_epipolar_distances(true_F->matrix(), pts1, pts2);

  // compute the fundamental matrix from the corresponding points
  std::vector<bool> inliers;
  fundamental_matrix_sptr F_sptr = est_f.estimate(pts1, pts2,
                                                inliers, 1.5);
  matrix_3x3d F = F_sptr->matrix();
  // check for sign difference
  if( true_F->matrix().cwiseProduct(F).sum() < 0.0 )
  {
    F *= -1;
  }

  // compare true and computed fundamental matrices
  std::cout << "true F = "<<*true_F<<std::endl;
  std::cout << "Estimated F = "<< F <<std::endl;
  TEST_NEAR("Fundamental Matrix Estimate", F, true_F->matrix(), 0.01);

  unsigned num_inliers = static_cast<unsigned>(std::count(inliers.begin(),
                                                          inliers.end(), true));
  std::cout << "num inliers "<<num_inliers<<std::endl;
  bool enough_inliers = num_inliers > pts1.size() / 2;
  TEST_EQUAL("Enough inliers", enough_inliers, true);
}



// test fundamental matrix estimation with outliers
IMPLEMENT_TEST(outlier_points)
{
  using namespace kwiver::maptk;
  ocv::estimate_fundamental_matrix est_f;

  // create landmarks at the random locations
  landmark_map_sptr landmarks = testing::init_landmarks(100);
  landmarks = testing::noisy_landmarks(landmarks, 1.0);

  // create a camera sequence (elliptical path)
  camera_map_sptr cameras = testing::camera_seq();

  // create tracks from the projections
  track_set_sptr tracks = projected_tracks(landmarks, cameras);

  // add random noise to track image locations
  tracks = testing::noisy_tracks(tracks, 0.5);

  const frame_id_t frame1 = 0;
  const frame_id_t frame2 = 10;

  camera_map::map_camera_t cams = cameras->cameras();
  camera_sptr cam1 = cams[frame1];
  camera_sptr cam2 = cams[frame2];
  camera_intrinsics_sptr cal1 = cam1->intrinsics();
  camera_intrinsics_sptr cal2 = cam2->intrinsics();

  // compute the true fundamental matrix from the cameras
  fundamental_matrix_sptr true_F = fundamental_matrix_from_cameras(*cam1, *cam2);

  // extract coresponding image points
  std::vector<track_sptr> trks = tracks->tracks();
  std::vector<vector_2d> pts1, pts2;
  for(unsigned int i=0; i<trks.size(); ++i)
  {
    if (i % 3 == 0)
    {
      pts1.push_back(testing::random_point2d(1000.0));
      pts2.push_back(testing::random_point2d(1000.0));
    }
    else
    { 
      pts1.push_back(trks[i]->find(frame1)->feat->loc());
      pts2.push_back(trks[i]->find(frame2)->feat->loc());
    }
  }

  print_epipolar_distances(true_F->matrix(), pts1, pts2);

  // compute the fundamental matrix from the corresponding points
  std::vector<bool> inliers;
  fundamental_matrix_sptr F_sptr = est_f.estimate(pts1, pts2,
                                                  inliers, 1.5);
  matrix_3x3d F = F_sptr->matrix();
  // check for sign difference
  if( true_F->matrix().cwiseProduct(F).sum() < 0.0 )
  {
    F *= -1;
  }

  // compare true and computed fundamental matrices
  std::cout << "true F = "<<*true_F<<std::endl;
  std::cout << "Estimated F = "<< F <<std::endl;
  TEST_NEAR("Fundamental Matrix Estimate", F, true_F->matrix(), 0.01);

  unsigned num_inliers = static_cast<unsigned>(std::count(inliers.begin(),
                                                          inliers.end(), true));
  std::cout << "num inliers "<<num_inliers<<" out of "<<pts1.size()<<" points."<<std::endl;
  bool enough_inliers = num_inliers > pts1.size() / 3;
  TEST_EQUAL("Enough inliers", enough_inliers, true);
}