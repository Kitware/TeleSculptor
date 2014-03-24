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
#include <boost/foreach.hpp>
#include <boost/math/constants/constants.hpp>
#include <maptk/core/camera.h>

#include <maptk/core/camera_io.h>

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

  double pi = boost::math::constants::pi<double>();
  camera_d a(vector_3d(-1, -1, -1),
             rotation_d(vector_4d(0, 0, 0, 1))),  // no rotation
           b(vector_3d(3, 3, 3),
             rotation_d(-pi / 2, vector_3d(0, 0, 1))),  // rotated around z-axis 90 degrees
           c;
  c = interpolate_camera(a, b, 0.5);

  cerr << "a.rotation: " << a.rotation().axis() << ' '  << a.rotation().angle() << endl;
  cerr << "b.rotation: " << b.rotation().axis() << ' '  << b.rotation().angle() << endl;

  cerr << "c.center  : " << c.center() << endl;
  TEST_NEAR("c.center.x", c.center().x(), 1, 1e-16);
  TEST_NEAR("c.center.y", c.center().y(), 1, 1e-16);
  TEST_NEAR("c.center.z", c.center().z(), 1, 1e-16);

  cerr << "c.rotation (aa): " << c.rotation().axis() << ' ' << c.rotation().angle() << endl;
  cerr << "c.rotation  (q): " << c.rotation() << endl;
  TEST_NEAR("c.rotation.axis.x", c.rotation().axis().x(),  0, 1e-15);
  TEST_NEAR("c.rotation.axis.y", c.rotation().axis().y(),  0, 1e-15);
  TEST_NEAR("c.rotation.axis.z", c.rotation().axis().z(), -1, 1e-15);
  TEST_NEAR("c.rotation.angle" , c.rotation().angle()   , pi / 4, 1e-15);
}


IMPLEMENT_TEST(multiple_interpolations)
{
  using namespace maptk;
  using namespace std;

  double pi = boost::math::constants::pi<double>();
  camera_d a(vector_3d(-1, -1, -1),
             rotation_d(vector_4d(0, 0, 0, 1))),        // no rotation
           b(vector_3d(3, 3, 3),
             rotation_d(-pi / 2, vector_3d(0, 0, 1)));  // rotated around z-axis 90 degrees
  vector<camera_d> cams;

  cams.push_back(a);
  interpolated_cameras(a, b, 3, cams);
  cams.push_back(b);

  cerr << "Vector size: " << cams.size() << endl;
  TEST_EQUAL("vector size", cams.size(), 5);
  BOOST_FOREACH(camera_d cam, cams)
  {
    cerr << "\t" << cam.center() << " :: " << cam.rotation().axis() << " " << cam.rotation().angle() << endl;
  }

  camera_d i1 = cams[1],
           i2 = cams[2],
           i3 = cams[3];
  cerr << "i1 .25 c : " << i1.center() << " :: " << i1.rotation().axis() << ' ' << i1.rotation().angle() << endl;
  cerr << "i2 .25 c : " << i2.center() << " :: " << i2.rotation().axis() << ' ' << i2.rotation().angle() << endl;
  cerr << "i3 .25 c : " << i3.center() << " :: " << i3.rotation().axis() << ' ' << i3.rotation().angle() << endl;

  TEST_NEAR("i1 center.x", i1.center().x(), 0, 1e-15);
  TEST_NEAR("i1 center.y", i1.center().y(), 0, 1e-15);
  TEST_NEAR("i1 center.z", i1.center().z(), 0, 1e-15);
  TEST_NEAR("i1 r.axis.x", i1.rotation().axis().x(), 0, 1e-15);
  TEST_NEAR("i1 r.axis.y", i1.rotation().axis().y(), 0, 1e-15);
  TEST_NEAR("i1 r.axis.z", i1.rotation().axis().z(), -1, 1e-15);
  TEST_NEAR("i1 r.angle",  i1.rotation().angle(), pi / 8, 1e-15);

  TEST_NEAR("i2 center.x", i2.center().x(), 1, 1e-15);
  TEST_NEAR("i2 center.y", i2.center().y(), 1, 1e-15);
  TEST_NEAR("i2 center.z", i2.center().z(), 1, 1e-15);
  TEST_NEAR("i2 r.axis.x", i2.rotation().axis().x(), 0, 1e-15);
  TEST_NEAR("i2 r.axis.y", i2.rotation().axis().y(), 0, 1e-15);
  TEST_NEAR("i2 r.axis.z", i2.rotation().axis().z(), -1, 1e-15);
  TEST_NEAR("i2 r.angle",  i2.rotation().angle(), pi / 4, 1e-15);

  TEST_NEAR("i3 center.x", i3.center().x(), 2, 1e-15);
  TEST_NEAR("i3 center.y", i3.center().y(), 2, 1e-15);
  TEST_NEAR("i3 center.z", i3.center().z(), 2, 1e-15);
  TEST_NEAR("i3 r.axis.x", i3.rotation().axis().x(), 0, 1e-15);
  TEST_NEAR("i3 r.axis.y", i3.rotation().axis().y(), 0, 1e-15);
  TEST_NEAR("i3 r.axis.z", i3.rotation().axis().z(), -1, 1e-15);
  TEST_NEAR("i3 r.angle",  i3.rotation().angle(), 3*pi / 8, 1e-15);
}


IMPLEMENT_TEST(x_axis_interpolation_cross)
{
  using namespace maptk;
  using namespace std;

  double pi = boost::math::constants::pi<double>();
  camera_d a(vector_3d(-1, -1, -1),
             rotation_d(pi / 4, vector_3d(0, 0, 1))),   // no rotation
           b(vector_3d(0, 0, 0),
             rotation_d(-pi / 4, vector_3d(0, 0, 1))),  // rotated around z-axis 90 degrees
           c(vector_3d(1, 1, 1),
             rotation_d(vector_4d(0, 0, 0, 1)));
  vector<camera_d> cams;

  //rotation_d a_r = a.rotation(),
  //           b_r = b.rotation(),
  //           c_r;
  //c_r = a_r.inverse() * b_r;
  //cerr << "a->b rotation axis angle: " << c_r.axis() << " " << c_r.angle() << endl;
  //TEST_NEAR("a->b angle under pi", c_r.angle(), pi / 2, 1e-15);
  //c_r = b_r.inverse() * a_r;
  //cerr << "b->a rotation axis angle: " << c_r.axis() << " " << c_r.angle() << endl;
  //TEST_NEAR("b->a angle under pi", c_r.angle(), pi / 2, 1e-15);

  //a = read_krtd_file("/home/purg/dev/MAP-TK/source/000005_028599_028599_20130626T152146.974326_S.krtd");
  //b = read_krtd_file("/home/purg/dev/MAP-TK/source/000005_028649_028649_20130626T152150.307458_S.krtd");
  //c_r = a.rotation().inverse() * b.rotation();
  //cerr << "a rotation axis : " << a.rotation().axis() << endl;
  //cerr << "a rotation angle: " << a.rotation().angle() << endl;
  //cerr << "b rotation axis : " << b.rotation().axis() << endl;
  //cerr << "b rotation angle: " << b.rotation().angle() << endl;
  //cerr << "krtd cams rotation axis : " << c_r.axis() << endl;
  //cerr << "krtd cams rotation angle: " << c_r.angle() << endl;
  //TEST_EQUAL("krtd angle rot angle", (c_r.angle() < pi) && (-pi < c_r.angle()), true);

  cams.push_back(a);
  interpolated_cameras(a, b, 2, cams);
  cams.push_back(b);

  cerr << "Vector size: " << cams.size() << endl;
  TEST_EQUAL("vector size", cams.size(), 4);
  BOOST_FOREACH(camera_d cam, cams)
  {
    cerr << "\t" << cam.center() << " :: " << cam.rotation().axis() << " " << cam.rotation().angle() << endl;
  }

  camera_d i1 = cams[1],
           i2 = cams[2];

  TEST_NEAR("i1 center.x", i1.center().x(), -2.0/3.0, 1e-15);
  TEST_NEAR("i1 center.y", i1.center().y(), -2.0/3.0, 1e-15);
  TEST_NEAR("i1 center.z", i1.center().z(), -2.0/3.0, 1e-15);
  TEST_NEAR("i1 r.axis.x", i1.rotation().axis().x(), 0, 1e-15);
  TEST_NEAR("i1 r.axis.y", i1.rotation().axis().y(), 0, 1e-15);
  TEST_NEAR("i1 r.axis.z", i1.rotation().axis().z(), 1, 1e-15);
  TEST_NEAR("i1 r.angle",  i1.rotation().angle(), pi / 12, 1e-15);

  TEST_NEAR("i2 center.x", i2.center().x(), -1.0/3.0, 1e-15);
  TEST_NEAR("i2 center.y", i2.center().y(), -1.0/3.0, 1e-15);
  TEST_NEAR("i2 center.z", i2.center().z(), -1.0/3.0, 1e-15);
  TEST_NEAR("i2 r.axis.x", i2.rotation().axis().x(), 0, 1e-15);
  TEST_NEAR("i2 r.axis.y", i2.rotation().axis().y(), 0, 1e-15);
  TEST_NEAR("i2 r.axis.z", i2.rotation().axis().z(), -1, 1e-15);
  TEST_NEAR("i2 r.angle",  i2.rotation().angle(), pi / 12, 1e-15);
}


// Full test case for above sub-test
