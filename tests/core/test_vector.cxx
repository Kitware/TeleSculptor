/*ckwg +5
 * Copyright 2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include <test_common.h>

#include <maptk/core/vector.h>

#define TEST_ARGS ()

DECLARE_TEST_MAP();

int
main(int argc, char* argv[])
{
  CHECK_ARGS(1);

  testname_t const testname = argv[1];

  RUN_TEST(testname);
}


IMPLEMENT_TEST(construct_2d)
{
  maptk::vector_2d v2d(10.0, 33.3);
  maptk::vector_2f v2f(5.0f, 4.5f);

  if (v2d.x() != 10.0 || v2f.x() != 5.0f)
  {
    TEST_ERROR("X coordinate of vector_2_ not initialized correctly");
  }

  if (v2d.y() != 33.3 || v2f.y() != 4.5f)
  {
    TEST_ERROR("Y coordinate of vector_2_ not initialized correctly");
  }

}


IMPLEMENT_TEST(construct_3d)
{
  maptk::vector_3d v3d(10.0, 33.3, 12.1);
  maptk::vector_3f v3f(5.0f, 4.5f, -6.3f);

  if (v3d.x() != 10.0 || v3f.x() != 5.0f)
  {
    TEST_ERROR("X coordinate of vector_3_ not initialized correctly");
  }

  if (v3d.y() != 33.3 || v3f.y() != 4.5f)
  {
    TEST_ERROR("Y coordinate of vector_3_ not initialized correctly");
  }

  if (v3d.z() != 12.1 || v3f.z() != -6.3f)
  {
    TEST_ERROR("Z coordinate of vector_3_ not initialized correctly");
  }
}


IMPLEMENT_TEST(construct_4d)
{
  maptk::vector_4d v4d(10.0, 33.3, 12.1, 0.0);
  maptk::vector_4f v4f(5.0f, 4.5f, -6.3f, 100.0f);

  if (v4d.x() != 10.0 || v4f.x() != 5.0f)
  {
    TEST_ERROR("X coordinate of vector_4_ not initialized correctly");
  }

  if (v4d.y() != 33.3 || v4f.y() != 4.5f)
  {
    TEST_ERROR("Y coordinate of vector_4_ not initialized correctly");
  }

  if (v4d.z() != 12.1 || v4f.z() != -6.3f)
  {
    TEST_ERROR("Z coordinate of vector_4_ not initialized correctly");
  }

  if (v4d.w() != 0.0 || v4f.w() != 100.0f)
  {
    TEST_ERROR("W coordinate of vector_4_ not initialized correctly");
  }
}
