/*ckwg +5
 * Copyright 2013 by Kitware, Inc. All Rights Reserved. Please refer to
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
