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

#include <boost/foreach.hpp>

#include <maptk/metrics.h>
#include <maptk/plugins/ceres/reprojection_error.h>
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


/// test the reprojection error
void
test_reprojection_error(const maptk::camera& cam,
                       const maptk::landmark& lm,
                       const maptk::feature& f)
{
  using namespace maptk;
  maptk::ceres::reprojection_error rpe(f.loc().x(), f.loc().y());

  double pose[6];
  vector_3d rot = cam.rotation().rodrigues();
  std::copy(rot.data(), rot.data()+3, pose);
  vector_3d center = cam.center();
  std::copy(center.data(), center.data()+3, pose+3);

  double intrinsics[5];
  camera_intrinsics_d K = cam.intrinsics();
  intrinsics[0] = K.focal_length();
  intrinsics[1] = K.principal_point().x();
  intrinsics[2] = K.principal_point().y();
  intrinsics[3] = K.aspect_ratio();
  intrinsics[4] = K.skew();

  double point[3] = {lm.loc().x(), lm.loc().y(), lm.loc().z()};

  vector_2d residuals;
  rpe(intrinsics, pose, point, residuals.data());

  TEST_NEAR("Residual near zero", residuals.norm(), 0.0, 1e-12);
}



// compare MAP-Tk camera projection to Ceres reprojection error models
IMPLEMENT_TEST(compare_projections)
{
  using namespace maptk;

  // create landmarks at the corners of a cube
  landmark_map_sptr landmarks = testing::cube_corners(2.0);
  landmark_map::map_landmark_t lm_map = landmarks->landmarks();

  // create a camera sequence (elliptical path)
  camera_map_sptr cameras = testing::camera_seq();
  camera_map::map_camera_t cam_map = cameras->cameras();

  // create tracks from the projections
  track_set_sptr tracks = projected_tracks(landmarks, cameras);
  std::vector<track_sptr> trks = tracks->tracks();

  double rmse = reprojection_rmse(cam_map, lm_map, trks);
  std::cout << "MAP-Tk reprojection RMSE: " << rmse << std::endl;
  TEST_NEAR("MAP-Tk reprojection RMSE should be small", rmse, 0.0, 1e-12);

  typedef std::map<landmark_id_t, landmark_sptr>::const_iterator lm_map_itr_t;
  typedef std::map<frame_id_t, camera_sptr>::const_iterator cam_map_itr_t;
  BOOST_FOREACH(const track_sptr& t, trks)
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
      test_reprojection_error(cam, lm, feat);
    }
  }
}
