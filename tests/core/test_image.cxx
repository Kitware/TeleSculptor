/*ckwg +5
 * Copyright 2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include <test_common.h>

#include <maptk/core/image.h>

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
  maptk::image img;
  if (img.size() != 0        ||
      img.first_pixel() != 0 ||
      img.width() != 0       ||
      img.height() != 0      ||
      img.depth() != 0       )
  {
    TEST_ERROR("The default image is not empty");
  }
}


IMPLEMENT_TEST(constructor)
{
  {
    maptk::image img(200,300);
    if (img.width() != 200  ||
        img.height() != 300 ||
        img.depth() != 1 )
    {
      TEST_ERROR("Constructed image does not have correct dimensions");
    }

    if (img.size() != 200*300)
    {
      TEST_ERROR("Constructor did not allocate correct amount of memory");
    }
  }
  {
    maptk::image img(200,300,3);
    if (img.width() != 200  ||
        img.height() != 300 ||
        img.depth() != 3 )
    {
      TEST_ERROR("Constructed image does not have correct dimensions");
    }

    if (img.size() != 200*300*3)
    {
      TEST_ERROR("Constructor did not allocate correct amount of memory");
    }
  }
}
