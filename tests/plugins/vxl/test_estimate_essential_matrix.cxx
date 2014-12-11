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

#include <maptk/projected_track_set.h>
#include <maptk/metrics.h>
#include <maptk/plugins/vxl/estimate_essential_matrix.h>
#include <maptk/plugins/vxl/register_algorithms.h>

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
  algo::estimate_essential_matrix_sptr est_e = algo::estimate_essential_matrix::create("vxl");
  if (!est_e)
  {
    TEST_ERROR("Unable to create vxl::estimate_essential_matrix by name");
  }
}


// Compute the essential matrix from cameras
maptk::matrix_3x3d
essential_matrix_from_cameras(const maptk::camera& right_cam,
                              const maptk::camera& left_cam)
{
  using namespace maptk;
  rotation_d R1 = right_cam.rotation();
  rotation_d R2 = left_cam.rotation();
  vector_3d t1 = right_cam.translation();
  vector_3d t2 = left_cam.translation();
  matrix_3x3d R(R2 * R1.inverse());
  matrix_3x3d tx = cross_product(t2 - R*t1);
  matrix_3x3d E(tx * R);
  E /= E.frobenius_norm();
  if (E(0,0) < 0)
  {
    E *= -1;
  }
  return E;
}


// Convert the essential matrix to a fundamental matrix
maptk::matrix_3x3d
essential_matrix_to_fundamental(const maptk::matrix_3x3d& E,
                                const maptk::camera_intrinsics_d& right_cal,
                                const maptk::camera_intrinsics_d& left_cal)
{
  using namespace maptk;
  matrix_3x3d Kr_inv = inverse(matrix_3x3d(right_cal));
  matrix_3x3d Kl_invt = inverse(matrix_3x3d(left_cal).transpose());
  return Kl_invt * E * Kr_inv;
}


// Print epipolar distance of pairs of points given a fundamental matrix
void print_epipolar_distances(const maptk::matrix_3x3d& F,
                              const std::vector<maptk::vector_2d> right_pts,
                              const std::vector<maptk::vector_2d> left_pts)
{
  using namespace maptk;
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
    double d = inner_product(vr, ll);
    std::cout <<" dist right = "<<d*sr<<"  dist left = "<<d*sl << std::endl;
  }
}


// test essential matrix estimation with ideal points
IMPLEMENT_TEST(ideal_points)
{
  using namespace maptk;
  vxl::estimate_essential_matrix est_e;

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
  camera_intrinsics_d cal1 = cam1->intrinsics();
  camera_intrinsics_d cal2 = cam2->intrinsics();

  // compute the true essential matrix from the cameras
  matrix_3x3d true_E = essential_matrix_from_cameras(*cam1, *cam2);

  // extract coresponding image points
  std::vector<track_sptr> trks = tracks->tracks();
  std::vector<vector_2d> pts1, pts2;
  for(unsigned int i=0; i<trks.size(); ++i)
  {
    pts1.push_back(trks[i]->find(frame1)->feat->loc());
    pts2.push_back(trks[i]->find(frame2)->feat->loc());
  }

  // print the epipolar distances using this essential matrix
  matrix_3x3d F = essential_matrix_to_fundamental(true_E, cal1, cal2);
  print_epipolar_distances(F, pts1, pts2);

  // compute the essential matrix from the corresponding points
  std::vector<bool> inliers;
  matrix_3x3d E = est_e.estimate(pts1, pts2, cal1, cal2, inliers, 1.5);
  E /= E.frobenius_norm();
  if (E(0,0) < 0)
  {
    E *= -1;
  }

  // compare true and computed essential matrices
  std::cout << "true E = "<<true_E<<std::endl;
  std::cout << "Estimated E = "<< E <<std::endl;
  TEST_NEAR("Essential Matrix Estimate", E, true_E, 1e-8);

  unsigned num_inliers = static_cast<unsigned>(std::count(inliers.begin(),
                                                          inliers.end(), true));
  std::cout << "num inliers "<<num_inliers<<std::endl;
  TEST_EQUAL("All points are inliers", num_inliers, pts1.size());
}


// test essential matrix estimation with noisy points
IMPLEMENT_TEST(noisy_points)
{
  using namespace maptk;
  vxl::estimate_essential_matrix est_e;

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
  camera_intrinsics_d cal1 = cam1->intrinsics();
  camera_intrinsics_d cal2 = cam2->intrinsics();

  // compute the true essential matrix from the cameras
  matrix_3x3d true_E = essential_matrix_from_cameras(*cam1, *cam2);

  // extract coresponding image points
  std::vector<track_sptr> trks = tracks->tracks();
  std::vector<vector_2d> pts1, pts2;
  for(unsigned int i=0; i<trks.size(); ++i)
  {
    pts1.push_back(trks[i]->find(frame1)->feat->loc());
    pts2.push_back(trks[i]->find(frame2)->feat->loc());
  }

  // print the epipolar distances using this essential matrix
  matrix_3x3d F = essential_matrix_to_fundamental(true_E, cal1, cal2);
  print_epipolar_distances(F, pts1, pts2);

  // compute the essential matrix from the corresponding points
  std::vector<bool> inliers;
  matrix_3x3d E = est_e.estimate(pts1, pts2, cal1, cal2, inliers, 1.5);
  E /= E.frobenius_norm();
  if (E(0,0) < 0)
  {
    E *= -1;
  }

  // compare true and computed essential matrices
  std::cout << "true E = "<<true_E<<std::endl;
  std::cout << "Estimated E = "<< E <<std::endl;
  TEST_NEAR("Essential Matrix Estimate", E, true_E, 0.01);

  unsigned num_inliers = static_cast<unsigned>(std::count(inliers.begin(),
                                                          inliers.end(), true));
  std::cout << "num inliers "<<num_inliers<<std::endl;
  bool enough_inliers = num_inliers > pts1.size() / 3;
  TEST_EQUAL("Enough inliers", enough_inliers, true);
}
