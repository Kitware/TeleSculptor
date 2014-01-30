/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include <test_common.h>

#include <maptk/core/rotation.h>

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
