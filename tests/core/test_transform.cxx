/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include <test_common.h>
#include <test_scene.h>

#include <maptk/core/transform.h>
#include <maptk/core/metrics.h>

#define TEST_ARGS ()

DECLARE_TEST_MAP();

int
main(int argc, char* argv[])
{
  CHECK_ARGS(1);

  testname_t const testname = argv[1];

  RUN_TEST(testname);
}


// add noise to landmarks before input to SBA
IMPLEMENT_TEST(cameras_landmarks)
{
  using namespace maptk;

  // create landmarks at the corners of a cube
  landmark_map_sptr landmarks = testing::cube_corners(2.0);

  // create a camera sequence (elliptical path)
  camera_map_sptr cameras = testing::camera_seq();

  // create tracks from the projections
  track_set_sptr tracks = testing::projected_tracks(landmarks, cameras);

  double rmse = reprojection_rmse(cameras->cameras(),
                                  landmarks->landmarks(),
                                  tracks->tracks());

  TEST_NEAR("sanitiy check: initial RMS error is zero",
            rmse, 0.0, 1e-14);

  similarity_d sim(22.4, rotation_d(vector_3d(0.1, -1.5, 2.0)),
                   vector_3d(100,-25,6));

  landmarks = transform(landmarks, sim);
  cameras = transform(cameras, sim);

  rmse = reprojection_rmse(cameras->cameras(),
                           landmarks->landmarks(),
                           tracks->tracks());

  TEST_NEAR("similarity transform does not change projection",
            rmse, 0.0, 1e-10);
}
