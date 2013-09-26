/*ckwg +5
 * Copyright 2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include <test_common.h>

#include <maptk/core/point_2d.h>

#define TEST_ARGS ()

DECLARE_TEST_MAP();

int
main(int argc, char* argv[])
{
  CHECK_ARGS(1);

  testname_t const testname = argv[1];

  RUN_TEST(testname);
}


IMPLEMENT_TEST(construct)
{
  maptk::point_2d ptd(10.0, 33.3);
  maptk::point_2f ptf(5.0f, 4.5f);

  if (ptd.x() != 10.0 || ptf.x() != 5.0f)
  {
    TEST_ERROR("X coordinate of point_2_ not initialized correctly");
  }

  if (ptd.y() != 33.3 || ptf.y() != 4.5f)
  {
    TEST_ERROR("Y coordinate of point_2_ not initialized correctly");
  }

}
