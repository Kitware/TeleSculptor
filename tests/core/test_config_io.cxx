/*ckwg +5
 * Copyright 2011-2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include <test_common.h>

#include <string>
#include <iostream>

#include <maptk/core/config_io.h>
#include <maptk/core/exceptions.h>
#include <maptk/core/types.h>

#include <boost/filesystem.hpp>

#define TEST_ARGS (maptk::path_t const& data_dir)
DECLARE_TEST_MAP();

int main(int argc, char* argv[])
{
  // expecting test name and data directory path
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

IMPLEMENT_TEST(read_test_config)
{
  maptk::config_t config = maptk::read_config_file(data_dir / "test_config_file.txt");
  TEST_EQUAL("foo:bar read",
             config->get_value<std::string>("foo:bar"),
             "baz");
  TEST_EQUAL("foo:things read",
             config->get_value<std::string>("foo:things"),
             "stuff");
  TEST_EQUAL("foo:sublevel:value read",
             config->get_value<std::string>("foo:sublevel:value"),
             "cool things and stuff");
  TEST_EQUAL("second_block:has read",
             config->get_value<std::string>("second_block:has"),
             "a value    with  spaces");
  TEST_NEAR("global_var read",
            config->get_value<float>("global_var"),
            3.14,
            0.000001);
  TEST_NEAR("global_var2 read",
            config->get_value<double>("global_var2"),
            1.12,
            0.000001);

  // extract sub-block, see that value access maintained
}
