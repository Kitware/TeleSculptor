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

#include <iostream>
#include <sstream>

#include <test_common.h>
#include <test_math.h>
#include <test_scene.h>

#include <vital/types/camera_map.h>
#include <vital/types/landmark_map.h>
#include <vital/types/track_set.h>
#include <vital/exceptions.h>


#include <maptk/projected_track_set.h>
#include <maptk/plugins/vxl/optimize_cameras.h>

#include <boost/foreach.hpp>


#define TEST_ARGS ()
DECLARE_TEST_MAP();

int main(int argc, char* argv[])
{
  CHECK_ARGS(1);
  kwiver::maptk::vxl::optimize_cameras::register_self();
  testname_t const testname = argv[1];
  RUN_TEST(testname);
}

using namespace kwiver::vital;

IMPLEMENT_TEST(creation)
{
  using namespace kwiver::maptk;
  algo::optimize_cameras_sptr cam_optimizer = algo::optimize_cameras::create("vxl");
  if (!cam_optimizer)
  {
    TEST_ERROR("Unable to create vxl::optimize_cameras by impl name.");
  }
}


IMPLEMENT_TEST(uninitialized)
{
  using namespace kwiver::maptk;
  using namespace std;

  camera_map_sptr cam_map;
  landmark_map_sptr lm_map;
  track_set_sptr trk_set;

  vxl::optimize_cameras optimizer;

  cerr << "cam_map before: " << cam_map << endl;

  EXPECT_EXCEPTION(
    kwiver::vital::invalid_value,
      optimizer.optimize(cam_map, trk_set, lm_map),
      "Running camera optimization with null input"
      );

  cerr << "cam_map after: " << cam_map << endl;

  TEST_EQUAL("cam_map", cam_map, 0);
}


IMPLEMENT_TEST(empty_input)
{
  using namespace kwiver::maptk;
  using namespace std;

  camera_map_sptr cam_map(new simple_camera_map());
  landmark_map_sptr lm_map(new simple_landmark_map());
  track_set_sptr trk_set(new simple_track_set());

  vxl::optimize_cameras optimizer;

  camera_map_sptr orig_map = cam_map;

  cerr << "cam_map before: " << cam_map << endl;
  optimizer.optimize(cam_map, trk_set, lm_map);
  cerr << "cam_map after : " << cam_map << endl;
  cerr << "orig map      : " << orig_map << endl;

  // make sure that a new camera map was created, but nothing was put in it.
  TEST_EQUAL("cam_map reference",
      cam_map == orig_map,
      false);
  TEST_EQUAL("cam_map size", cam_map->size(), 0);
  TEST_EQUAL("orig map size", orig_map->size(), 0);
}


IMPLEMENT_TEST(no_noise)
{
  using namespace kwiver::maptk;
  using namespace std;

  // Create cameras, landmarks and tracks.
  // Optimize already optimimal elements to make sure they don't get changed
  // much.

  camera_map::map_camera_t original_cams = testing::camera_seq()->cameras();

  landmark_map_sptr landmarks = testing::cube_corners(2.0);
  camera_map_sptr working_cam_map(new simple_camera_map(original_cams));
  track_set_sptr tracks = projected_tracks(landmarks,
                                                  working_cam_map);

  vxl::optimize_cameras optimizer;
  optimizer.optimize(working_cam_map, tracks, landmarks);

  vector_3d zero_3d_vec(0,0,0);
  vector_4d zero_4d_vec(0,0,0,0);
  matrix_3x3d zero_mat = matrix_3x3d::Zero();
  ostringstream ss;

  double ep = 1e-14;
  BOOST_FOREACH(camera_map::map_camera_t::value_type const& p,
                working_cam_map->cameras())
  {
    // difference in camera center
    vector_3d a_c = p.second->center(),
              b_c = original_cams[p.first]->center(),
              c_c;
    c_c = a_c - b_c;
    //cerr << "frm[" << p.first << "]\t:: center delta     :: " << c_c << endl;
    ss.str("");
    ss << "frm[" << p.first << "] center delta check";
    TEST_NEAR(ss.str(), c_c, zero_3d_vec, ep);

    // difference in camera rotation
    rotation_d a_r = p.second->rotation(),
               b_r = original_cams[p.first]->rotation();
    vector_4d c_r = a_r.quaternion().coeffs() - b_r.quaternion().coeffs();
    //cerr << "frm[" << p.first << "]\t:: quaternion delta :: " << c_r << endl;
    ss.str("");
    ss << "frm[" << p.first << "] quaternion delta check";
    TEST_NEAR(ss.str(), c_r, zero_4d_vec, ep);

    // difference in camera intrinsics
    camera_intrinsics_d a_k = p.second->intrinsics(),
                        b_k = original_cams[p.first]->intrinsics();
    matrix_3x3d c_k = matrix_3x3d(a_k) - matrix_3x3d(b_k);
    //cerr << "frm[" << p.first << "]\t:: intrinsics delta :: " << c_k << endl;
    ss.str("");
    ss << "frm[" << p.first << "] intrinsics";
    TEST_NEAR(ss.str(), c_k, zero_mat, ep);
  }
}


IMPLEMENT_TEST(noisy_cameras)
{
  using namespace kwiver::maptk;
  using namespace std;

  // Same as above, but create an analogous set of cameras with noise added.
  // Check that optimized cameras are close to the original cameras.

  camera_map::map_camera_t original_cams = testing::camera_seq()->cameras();

  landmark_map_sptr landmarks = testing::cube_corners(2.0);
  camera_map_sptr working_cam_map(new simple_camera_map(original_cams));
  track_set_sptr tracks = projected_tracks(landmarks, working_cam_map);

  working_cam_map = testing::noisy_cameras(working_cam_map, 0.1, 0.1);

  vxl::optimize_cameras optimizer;
  optimizer.optimize(working_cam_map, tracks, landmarks);

  vector_3d zero_3d_vec(0,0,0);
  vector_4d zero_4d_vec(0,0,0,0);
  matrix_3x3d zero_mat = matrix_3x3d::Zero();
  ostringstream ss;

  double ep = 2e-10;
  BOOST_FOREACH(camera_map::map_camera_t::value_type const& p,
                working_cam_map->cameras())
  {
    // difference in camera center
    vector_3d a_c = p.second->center(),
              b_c = original_cams[p.first]->center(),
              c_c;
    c_c = a_c - b_c;
    //cerr << "frm[" << p.first << "]\t:: center delta     :: " << c_c << endl;
    ss.str("");
    ss << "frm[" << p.first << "] center delta check";
    TEST_NEAR(ss.str(), c_c, zero_3d_vec, ep);

    // difference in camera rotation
    rotation_d a_r = p.second->rotation(),
               b_r = original_cams[p.first]->rotation();
    vector_4d c_r = a_r.quaternion().coeffs() - b_r.quaternion().coeffs();
    //cerr << "frm[" << p.first << "]\t:: quaternion delta :: " << c_r << endl;
    ss.str("");
    ss << "frm[" << p.first << "] quaternion delta check";
    TEST_NEAR(ss.str(), c_r, zero_4d_vec, ep);

    // difference in camera intrinsics
    camera_intrinsics_d a_k = p.second->intrinsics(),
                        b_k = original_cams[p.first]->intrinsics();
    matrix_3x3d c_k = matrix_3x3d(a_k) - matrix_3x3d(b_k);
    //cerr << "frm[" << p.first << "]\t:: intrinsics delta :: " << c_k << endl;
    ss.str("");
    ss << "frm[" << p.first << "] intrinsics";
    TEST_NEAR(ss.str(), c_k, zero_mat, ep);
  }
}
