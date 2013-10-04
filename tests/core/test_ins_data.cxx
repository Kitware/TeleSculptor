/*ckwg +5
 * Copyright 2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include <test_common.h>

#include <sstream>
#include <iostream>
#include <maptk/core/ins_data.h>

#define TEST_ARGS ()

DECLARE_TEST_MAP();

int
main(int argc, char* argv[])
{
  CHECK_ARGS(1);

  testname_t const testname = argv[1];

  RUN_TEST(testname);
}

static bool test_iostream(const maptk::ins_data& d)
{
  std::cout << "Sample INS data:\n" << d << std::flush;
  std::stringstream ss;
  ss << d;

  // read the data back in
  maptk::ins_data d2;
  ss >> d2;
  std::cout << "Re-read data as:\n" << d2;

  return d == d2;
}


IMPLEMENT_TEST(iostream_low_prec)
{
  maptk::ins_data d(28.2, -37.1, 0.034,
                    47.9, 82.0, 170.0,
                    "TEST",
                    23.0, 345,
                    0.0, 2.5, 3.0,
                    3, 8, -1);
  if( !test_iostream(d) )
  {
    TEST_ERROR("Written INS data does not match what is read back in.");
  }
}


IMPLEMENT_TEST(iostream_high_prec)
{
  maptk::ins_data d(28.29342, -37.19432, 0.00000349,
                    47.9347286349, 82.033292098, 1700.0,
                    "MAPTK",
                    23923423.003, 345,
                    0.0, 2.5, 3.0,
                    3, 8, -1);
  if( !test_iostream(d) )
  {
    TEST_ERROR("Written INS data does not match what is read back in.");
  }
}
