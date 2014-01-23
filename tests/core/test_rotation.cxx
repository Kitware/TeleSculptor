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
