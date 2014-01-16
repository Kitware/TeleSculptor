/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include <test_common.h>

#include <maptk/core/camera.h>

#define TEST_ARGS ()

DECLARE_TEST_MAP();

int
main(int argc, char* argv[])
{
  CHECK_ARGS(1);

  testname_t const testname = argv[1];

  RUN_TEST(testname);
}


IMPLEMENT_TEST(look_at)
{
  using namespace maptk;
  vector_2d pp(300,400);
  camera_intrinsics_d K(1000, pp);
  vector_3d focus(0, 1, -2);
  maptk::camera_d cam(vector_3d(3, -4, 7), rotation_d(), K);
  cam.look_at(focus);

  vector_2d ifocus = cam.project(focus);
  TEST_NEAR("look_at focus projects to origin",
            (ifocus-pp).magnitude(), 0.0, 1e-12);

  vector_2d ifocus_up = cam.project(focus + vector_3d(0,0,2));
  vector_2d tmp = ifocus_up - pp;
  TEST_NEAR("look_at vertical projects vertical",
            tmp.x(), 0.0, 1e-12);
  // "up" in image space is actually negative Y because the
  // Y axis is inverted
  TEST_EQUAL("look_at up projects up", tmp.y() < 0.0, true);
}
