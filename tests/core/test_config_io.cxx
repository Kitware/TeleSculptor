/*ckwg +5
 * Copyright 2011-2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include <test_common.h>

#include <iostream>

#include <maptk/core/config_io.h>
#include <maptk/core/exceptions.h>
#include <maptk/core/types.h>

#include <boost/filesystem.hpp>

#define TEST_ARGS (maptk::path_t data_dir)
DECLARE_TEST_MAP();

int main(int argc, char* argv[])
{
  // expecting test name and data directory path`
  CHECK_ARGS(2);
  testname_t const testname = argv[1];
  maptk::path_t data_dir(argv[2]);
  RUN_TEST(testname, data_dir);
}

IMPLEMENT_TEST(config_path_not_exist)
{
  maptk::path_t fp("/this/shouldnt/exist/anywhere");

  EXPECT_EXCEPTION(
    maptk::file_not_found_exception,
    maptk::read_config_file(fp),
    "calling config read with non-existant file"
  );
}

IMPLEMENT_TEST(config_path_not_file)
{
  maptk::path_t fp = boost::filesystem::current_path();

  EXPECT_EXCEPTION(
    maptk::file_not_found_exception,
    maptk::read_config_file(fp),
    "calling config read with directory path as argument"
  );
}
