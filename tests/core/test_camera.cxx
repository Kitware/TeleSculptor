/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief test core camera class
 */

#include <test_common.h>

#include <iostream>
#include <boost/math/constants/constants.hpp>
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


IMPLEMENT_TEST(projection)
{
  using namespace maptk;
  vector_2d pp(300,400);
  camera_intrinsics_d K(1000, pp);
  vector_3d focus(0, 1, -2);
  maptk::camera_d cam(vector_3d(3, -4, 7), rotation_d(), K);
  cam.look_at(focus);

  matrix_3x4d P(cam);
  vector_3d test_pt(1,2,3);
  vector_4d test_hpt(test_pt.x(), test_pt.y(), test_pt.z(), 1.0);

  vector_3d proj_hpt = P * test_hpt;
  vector_2d proj_pt(proj_hpt.x()/proj_hpt.z(), proj_hpt.y()/proj_hpt.z());

  TEST_NEAR("camera projection = matrix multiplication",
             (cam.project(test_pt) - proj_pt).magnitude(), 0.0, 1e-12);
}


IMPLEMENT_TEST(interpolation)
{
  using namespace maptk;
  using namespace std;

  camera_d a(vector_3d(-1, -1, -1),
             rotation_d(vector_4d(0, 0, 0, 1))),  // no rotation
           b(vector_3d(3, 3, 3),
             rotation_d(vector_4d(0, 1, 0, 0))),  // flipped over y-axis
           c;
  c = interpolate_camera(a, b, 0.5);
  double pi = boost::math::constants::pi<double>();

  cerr << "c.center  : " << c.center() << endl;
  TEST_NEAR("c.center.x", c.center().x(), 1, 1e-16);
  TEST_NEAR("c.center.y", c.center().y(), 1, 1e-16);
  TEST_NEAR("c.center.z", c.center().z(), 1, 1e-16);

  cerr << "c.rotation: " << c.rotation().axis() << ' ' << c.rotation().angle() << endl;
  TEST_NEAR("c.rotation.axis.x", c.rotation().axis().x(), 0, 1e-16);
  TEST_NEAR("c.rotation.axis.x", c.rotation().axis().x(), 0, 1e-16);
  TEST_NEAR("c.rotation.axis.x", c.rotation().axis().x(), 0, 1e-16);
  TEST_NEAR("c.rotation.angle" , c.rotation().angle()   , pi / 2, 1e-16);
}
