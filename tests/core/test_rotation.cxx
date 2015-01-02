/*ckwg +29
 * Copyright 2013-2014 by Kitware, Inc.
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
 * \brief test core rotation class
 */

#include <test_common.h>

#include <iostream>
#include <vector>

#include <maptk/rotation.h>

#include <boost/foreach.hpp>
#include <boost/math/constants/constants.hpp>


#define TEST_ARGS ()

DECLARE_TEST_MAP();

int
main(int argc, char* argv[])
{
  CHECK_ARGS(1);

  testname_t const testname = argv[1];

  RUN_TEST(testname);
}


IMPLEMENT_TEST(default_constructor)
{
  maptk::rotation_d rot;
  if (rot.quaternion() != maptk::vector_4d(0,0,0,1))
  {
    TEST_ERROR("The default rotation is not the identity");
  }
}


IMPLEMENT_TEST(convert_matrix)
{
  maptk::rotation_d rot;
  maptk::matrix_3x3d R = maptk::matrix_3x3d(rot);
  maptk::matrix_3x3d I = maptk::matrix_3x3d().set_identity();
  std::cout << R << I << std::endl;

  if (R != I)
  {
    TEST_ERROR("Default rotation is not identity matrix");
  }

}


IMPLEMENT_TEST(convert_rodrigues)
{
  using namespace maptk;

  rotation_d rot;
  TEST_EQUAL("Rodrigues vector of identity is zero",
              rot.rodrigues().magnitude(), 0.0);

  rotation_d rot2(vector_3d(0,0,0));
  TEST_EQUAL("Rotation from zero Rodrigues vector is identity",
              rotation_d(vector_3d(0,0,0)),
              rotation_d());

  vector_3d rvec(2,-1,0.5);
  rotation_d rot3(rvec);
  using namespace std;
  cerr << "rvec magnitude: " << rvec.magnitude() << endl;
  cerr << "rot3 magnitude: " << rot3.rodrigues().magnitude() << endl;
  TEST_NEAR("Rodrigues convert to and from rotation is same",
            (rot3.rodrigues() - rvec).magnitude(), 0.0, 1e-14);

  TEST_NEAR("Rodrigues direction == rotation axis",
            (normalized(rvec) - rot3.axis()).magnitude(), 0.0, 1e-14);
}


IMPLEMENT_TEST(convert_axis_angle)
{
  using namespace maptk;

  double angle = 0.8;
  vector_3d axis = normalized(vector_3d(-3,2,1));

  rotation_d rot(angle, axis);

  TEST_NEAR("Angle extracted from rotation matches input",
            rot.angle(), angle, 1e-14);

  TEST_NEAR("Axis extracted from rotation matches input",
            (rot.axis() - axis).magnitude(), 0.0, 1e-14);
}


IMPLEMENT_TEST(convert_yaw_pitch_roll)
{
  using namespace maptk;

  const double yaw = 1.2;
  const double pitch = 0.3;
  const double roll = -1.0;
  double y,p,r;

  {
    rotation_d rot(yaw, pitch, roll);
    std::cout <<"(xxx) rotation mat:\n"<<matrix_3x3d(rot)<<std::endl;
    rot.get_yaw_pitch_roll(y, p, r);

    TEST_NEAR("(xxx) Yaw angle", y, yaw, 1e-14);
    TEST_NEAR("(xxx) Pitch angle", p, pitch, 1e-14);
    TEST_NEAR("(xxx) Roll angle", r, roll, 1e-14);
  }
  {
    rotation_d rot(0, pitch, roll);
    std::cout <<"(0xx) rotation mat:\n"<<matrix_3x3d(rot)<<std::endl;
    rot.get_yaw_pitch_roll(y, p, r);

    TEST_NEAR("(0xx) Yaw angle", y, 0, 1e-14);
    TEST_NEAR("(0xx) Pitch angle", p, pitch, 1e-14);
    TEST_NEAR("(0xx) Roll angle", r, roll, 1e-14);
  }
  {
    rotation_d rot(yaw, 0, roll);
    std::cout <<"(x0x) rotation mat:\n"<<matrix_3x3d(rot)<<std::endl;
    rot.get_yaw_pitch_roll(y, p, r);

    TEST_NEAR("(x0x) Yaw angle", y, yaw, 1e-14);
    TEST_NEAR("(x0x) Pitch angle", p, 0, 1e-14);
    TEST_NEAR("(x0x) Roll angle", r, roll, 1e-14);
  }
  {
    rotation_d rot(yaw, pitch, 0);
    std::cout <<"(xx0) rotation mat:\n"<<matrix_3x3d(rot)<<std::endl;
    rot.get_yaw_pitch_roll(y, p, r);

    TEST_NEAR("(xx0) Yaw angle", y, yaw, 1e-14);
    TEST_NEAR("(xx0) Pitch angle", p, pitch, 1e-14);
    TEST_NEAR("(xx0) Roll angle", r, 0, 1e-14);
  }
  {
    rotation_d rot(0, 0, roll);
    std::cout <<"(00x) rotation mat:\n"<<matrix_3x3d(rot)<<std::endl;
    rot.get_yaw_pitch_roll(y, p, r);

    TEST_NEAR("(00x) Yaw angle", y, 0, 1e-14);
    TEST_NEAR("(00x) Pitch angle", p, 0, 1e-14);
    TEST_NEAR("(00x) Roll angle", r, roll, 1e-14);
  }
  {
    rotation_d rot(0, pitch, 0);
    std::cout <<"(0x0) rotation mat:\n"<<matrix_3x3d(rot)<<std::endl;
    rot.get_yaw_pitch_roll(y, p, r);

    TEST_NEAR("(0x0) Yaw angle", y, 0, 1e-14);
    TEST_NEAR("(0x0) Pitch angle", p, pitch, 1e-14);
    TEST_NEAR("(0x0) Roll angle", r, 0, 1e-14);
  }
  {
    rotation_d rot(yaw, 0, 0);
    std::cout <<"(x00) rotation mat:\n"<<matrix_3x3d(rot)<<std::endl;
    rot.get_yaw_pitch_roll(y, p, r);

    TEST_NEAR("(x00) Yaw angle", y, yaw, 1e-14);
    TEST_NEAR("(x00) Pitch angle", p, 0, 1e-14);
    TEST_NEAR("(x00) Roll angle", r, 0, 1e-14);
  }
  {
    rotation_d rot(0, 0, 0);
    std::cout <<"(000) rotation mat:\n"<<matrix_3x3d(rot)<<std::endl;
    rot.get_yaw_pitch_roll(y, p, r);

    TEST_NEAR("(000) Yaw angle", y, 0, 1e-14);
    TEST_NEAR("(000) Pitch angle", p, 0, 1e-14);
    TEST_NEAR("(000) Roll angle", r, 0, 1e-14);
  }
}


IMPLEMENT_TEST(compose)
{
  using namespace maptk;

  rotation_d rot1(vector_3d(0.1, -1.5, 2.0));
  rotation_d rot2(vector_3d(-0.5, -0.5, 1.0));

  matrix_3x3d quat_comp_rot = rot1 * rot2;
  matrix_3x3d mat_comp_rot = matrix_3x3d(rot1) * matrix_3x3d(rot2);
  std::cout << "quaternion composition: "<<quat_comp_rot<<std::endl;
  std::cout << "matrix composition: "<<mat_comp_rot<<std::endl;

  TEST_NEAR("Matrix multiplication matches quaternion composition",
            (quat_comp_rot - mat_comp_rot).frobenius_norm(),
            0.0, 1e-14);
}


IMPLEMENT_TEST(interpolation)
{
  using namespace maptk;

  double pi = boost::math::constants::pi<double>();
  rotation_d x(0, vector_3d(1, 0, 0)),
             y(pi / 2, vector_3d(0, 1, 0)),
             z;
  z = interpolate_rotation(x, y, 0.5);

  using namespace std;
  cerr << "x: " << x.axis() << " " << x.angle() << endl
       << "y: " << y.axis() << " " << y.angle() << endl
       << "z: " << z.axis() << " " << z.angle() << endl;

  TEST_NEAR("z-axis 0", z.axis()[0], 0, 1e-15);
  TEST_NEAR("z-axis 1", z.axis()[1], 1, 1e-15);
  TEST_NEAR("z-axis 2", z.axis()[2], 0, 1e-15);
  TEST_NEAR("z-angle",  z.angle(), pi / 4, 1e-15);
}


IMPLEMENT_TEST(multiple_interpolations)
{
  using namespace maptk;
  using namespace std;

  double pi = boost::math::constants::pi<double>();
  rotation_d x(0, vector_3d(1, 0, 0)),
             y(pi / 2, vector_3d(0, 1, 0));
  vector<rotation_d> rots;

  rots.push_back(x);
  interpolated_rotations(x, y, 3, rots);
  rots.push_back(y);

  cerr << "Vector size: " << rots.size() << endl;
  TEST_EQUAL("vector size", rots.size(), 5);
  BOOST_FOREACH(rotation_d rot, rots)
  {
    cerr << "\t" << rot.axis() << ' ' << rot.angle() << endl;
  }

  rotation_d i1 = rots[1],
             i2 = rots[2],
             i3 = rots[3];
  cerr << "i1 .25 : " << i1.axis() << ' ' << i1.angle() << endl;
  cerr << "i2 .50 : " << i2.axis() << ' ' << i2.angle() << endl;
  cerr << "i3 .75 : " << i3.axis() << ' ' << i3.angle() << endl;

  TEST_NEAR("i1 axis x", i1.axis().x(), 0, 1e-15);
  TEST_NEAR("i1 axis y", i1.axis().y(), 1, 1e-15);
  TEST_NEAR("i1 axis z", i1.axis().z(), 0, 1e-15);
  TEST_NEAR("i1 andgle", i1.angle(), pi / 8, 1e-15);

  TEST_NEAR("i2 axis x", i2.axis().x(), 0, 1e-15);
  TEST_NEAR("i2 axis y", i2.axis().y(), 1, 1e-15);
  TEST_NEAR("i2 axis z", i2.axis().z(), 0, 1e-15);
  TEST_NEAR("i2 andgle", i2.angle(), pi / 4, 1e-15);

  TEST_NEAR("i3 axis x", i3.axis().x(), 0, 1e-15);
  TEST_NEAR("i3 axis y", i3.axis().y(), 1, 1e-15);
  TEST_NEAR("i3 axis z", i3.axis().z(), 0, 1e-15);
  TEST_NEAR("i3 andgle", i3.angle(), (3*pi) / 8, 1e-15);
}
