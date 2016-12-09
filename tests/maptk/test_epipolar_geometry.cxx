/*ckwg +29
 * Copyright 2015-2016 by Kitware, Inc.
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

#include <arrows/core/projected_track_set.h>
#include <arrows/core/epipolar_geometry.h>


#define TEST_ARGS ()

DECLARE_TEST_MAP();

int
main(int argc, char* argv[])
{
  CHECK_ARGS(1);

  testname_t const testname = argv[1];

  RUN_TEST(testname);
}

using namespace kwiver::vital;


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


// test essential matrix estimation with ideal points
IMPLEMENT_TEST(ideal_points)
{
  using namespace kwiver::maptk;

  // create landmarks at the random locations
  landmark_map_sptr landmarks = testing::init_landmarks(100);
  landmarks = testing::noisy_landmarks(landmarks, 1.0);

  // create a camera sequence (elliptical path)
  camera_map_sptr cameras = testing::camera_seq();

  // create tracks from the projections
  track_set_sptr tracks = kwiver::arrows::projected_tracks(landmarks, cameras);

  const frame_id_t frame1 = 0;
  const frame_id_t frame2 = 10;

  camera_map::map_camera_t cams = cameras->cameras();
  camera_sptr cam1 = cams[frame1];
  camera_sptr cam2 = cams[frame2];
  camera_intrinsics_sptr cal1 = cam1->intrinsics();
  camera_intrinsics_sptr cal2 = cam2->intrinsics();

  // compute the true essential matrix from the cameras
  essential_matrix_sptr em = kwiver::arrows::essential_matrix_from_cameras(*cam1, *cam2);
  fundamental_matrix_sptr fm = kwiver::arrows::fundamental_matrix_from_cameras(*cam1, *cam2);

  // extract coresponding image points
  std::vector<track_sptr> trks = tracks->tracks();
  std::vector<vector_2d> pts1, pts2;
  for(unsigned int i=0; i<trks.size(); ++i)
  {
    pts1.push_back(trks[i]->find(frame1)->feat->loc());
    pts2.push_back(trks[i]->find(frame2)->feat->loc());
  }

  std::vector<vector_2d> norm_pts1, norm_pts2;
  VITAL_FOREACH(const vector_2d& p, pts1)
  {
    norm_pts1.push_back(cal1->unmap(p));
  }
  VITAL_FOREACH(const vector_2d& p, pts2)
  {
    norm_pts2.push_back(cal2->unmap(p));
  }

  // print the epipolar distances using this fundamental matrix
  print_epipolar_distances(fm->matrix(), pts1, pts2);

  // compute the inliers with a small scale
  std::vector<bool> inliers = kwiver::arrows::mark_fm_inliers(*fm, pts1, pts2, 1e-8);

  unsigned num_inliers = static_cast<unsigned>(std::count(inliers.begin(),
                                                          inliers.end(), true));
  std::cout << "fundamental matrix - num inliers "<<num_inliers<<std::endl;
  TEST_EQUAL("All points are inliers", num_inliers, pts1.size());

  // compute the inliers with a small scale
  inliers = kwiver::arrows::mark_fm_inliers(fundamental_matrix_d(em->matrix()),
                                            norm_pts1, norm_pts2, 1e-8);

  num_inliers = static_cast<unsigned>(std::count(inliers.begin(),
                                                 inliers.end(), true));
  std::cout << "essential matrix - num inliers "<<num_inliers<<std::endl;
  TEST_EQUAL("All points are inliers", num_inliers, norm_pts1.size());
}
