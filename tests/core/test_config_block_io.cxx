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
 * \brief core config_block_io tests
 */

#include <test_common.h>

#include <string>
#include <iostream>

#include <maptk/config_block_io.h>
#include <maptk/exceptions.h>
#include <maptk/types.h>

#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>

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

#define print_config(config) \
  BOOST_FOREACH( config_block_key_t key, config->available_values() ) \
  { \
    std::cerr << "\t" \
              << key << " = " << config->get_value<config_block_key_t>(key) \
              << std::endl; \
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

IMPLEMENT_TEST(successful_config_read)
{
  maptk::config_block_sptr config = maptk::read_config_file(data_dir / "test_config-valid_file.txt");

  using std::cerr;
  using std::endl;
  cerr << "Available keys in the config_block:" << endl;
  BOOST_FOREACH(maptk::config_block_key_t key, config->available_values())
  {
    cerr << "\t\"" << key << "\" := \"" << config->get_value<std::string>(key) << "\"" << endl;
  }

  TEST_EQUAL("num config params",
             config->available_values().size(),
             8);
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
  TEST_EQUAL("second_block:mode read",
             config->get_value<std::string>("second_block:more"),
             "has a trailing comment");
  TEST_NEAR("global_var read",
            config->get_value<float>("global_var"),
            3.14,
            0.000001);
  TEST_NEAR("global_var2 read",
            config->get_value<double>("global_var2"),
            1.12,
            0.000001);
  TEST_EQUAL("tabbed:value read",
             config->get_value<std::string>("tabbed:value"),
             "should be valid");

  // extract sub-block, see that value access maintained
  maptk::config_block_sptr foo_subblock = config->subblock_view("foo");
  TEST_EQUAL("foo subblock bar read",
             foo_subblock->get_value<std::string>("bar"),
             "baz");
  TEST_EQUAL("foo subblock sublevel read",
             foo_subblock->get_value<std::string>("sublevel:value"),
             "cool things and stuff");
  TEST_EQUAL("foo nested extraction",
             config->subblock_view("foo")->subblock_view("sublevel")
                                         ->get_value<std::string>("value"),
             "cool things and stuff");
}

IMPLEMENT_TEST(successful_config_read_named_block)
{
  maptk::config_block_sptr config = maptk::read_config_file(data_dir / "test_config-valid_file.txt",
                                                         "block_name_here");

  using std::cerr;
  using std::endl;
  cerr << "Available keys in the config_block:" << endl;
  BOOST_FOREACH(maptk::config_block_key_t key, config->available_values())
  {
    cerr << "\t\"" << key << "\" := \"" << config->get_value<std::string>(key) << "\"" << endl;
  }

  TEST_EQUAL("num config params",
             config->available_values().size(),
             8);
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
  TEST_EQUAL("second_block:mode read",
             config->get_value<std::string>("second_block:more"),
             "has a trailing comment");
  TEST_NEAR("global_var read",
            config->get_value<float>("global_var"),
            3.14,
            0.000001);
  TEST_NEAR("global_var2 read",
            config->get_value<double>("global_var2"),
            1.12,
            0.000001);
  TEST_EQUAL("tabbed:value read",
             config->get_value<std::string>("tabbed:value"),
             "should be valid");
}

IMPLEMENT_TEST(invalid_config_file)
{
  EXPECT_EXCEPTION(
      maptk::file_not_parsed_exception,
      maptk::read_config_file(data_dir / "test_config-invalid_file.txt"),
      "calling config_block read on badly formatted file"
      );
}

IMPLEMENT_TEST(invalid_keypath)
{
  EXPECT_EXCEPTION(
      maptk::file_not_parsed_exception,
      maptk::read_config_file(data_dir / "test_config-invalid_keypath.txt"),
      "read attempt on file with invalid key path"
      );
}

IMPLEMENT_TEST(config_with_comments)
{
  maptk::config_block_sptr config = maptk::read_config_file(data_dir / "test_config-comments.txt");

  using std::string;

  TEST_EQUAL("num config params",
             config->available_values().size(),
             4);

  TEST_EQUAL("general:logging param",
             config->get_value<string>("general:logging"),
             "on");
  TEST_EQUAL("general:another_var param",
             config->get_value<string>("general:another_var"),
             "foo");
  TEST_EQUAL("general:yet_more param",
             config->get_value<string>("general:yet_more"),
             "bar");
  TEST_EQUAL("final:value param",
             config->get_value<string>("final:value"),
             "things and stuff");
}

IMPLEMENT_TEST(write_config_simple_success)
{
  using namespace maptk;
  using namespace std;
  namespace bfs = boost::filesystem;

  config_block_sptr orig_config = config_block::empty_config("simple_test");

  config_block_key_t keyA = config_block_key_t("test_key_1");
  config_block_key_t keyB = config_block_key_t("test_key_2");
  config_block_key_t keyC = config_block_key_t("test_key_3");
  config_block_key_t keyD = config_block_key_t("test_key_4");
  config_block_key_t keyE = config_block_key_t("test_key_5");
  config_block_key_t keyF = config_block_key_t("test_key_6");
  config_block_key_t keyG = config_block_key_t("test_key_7");

  config_block_value_t valueA = config_block_value_t("test_value_a");
  config_block_value_t valueB = config_block_value_t("test_value_b");
  config_block_value_t valueC = config_block_value_t("test_value_c");
  config_block_value_t valueD = config_block_value_t("test_value_d");
  config_block_value_t valueE = config_block_value_t("test_value_e");
  config_block_value_t valueF = config_block_value_t("test_value_f");
  config_block_value_t valueG = config_block_value_t("test_value_g");

  config_block_description_t descrD = config_block_description_t("Test descr 1");
  config_block_description_t descrE = config_block_description_t(
      "This is a really long description that should probably span multiple "
      "lines because it exceeds the defined character width we would like "
      "in order to make output files more readable."
      );
  config_block_description_t descrF = config_block_description_t(
      "this is a comment\n"
      "that has manual new-line splits\n"
      "that should be preserved\n"
      "\n"
      "Pretend list:\n"
      "  - foo\n"
      "    - bar\n"
      );
  config_block_description_t descrG = config_block_description_t(
      "This has a # in it"
      );

  config_block_key_t subblock_name = config_block_key_t("subblock");
  config_block_sptr subblock = orig_config->subblock_view(subblock_name);

  orig_config->set_value(keyA, valueA);
  orig_config->set_value(keyB, valueB);
  subblock->set_value(keyC, valueC);
  orig_config->set_value(keyD, valueD, descrD);
  orig_config->set_value(keyE, valueE, descrE);
  orig_config->set_value(keyF, valueF, descrF);
  orig_config->set_value(keyG, valueG, descrG);

  cerr << "ConfigBlock for writing:" << endl;
  print_config(orig_config);

  path_t base_path = bfs::temp_directory_path() / bfs::unique_path();
  cerr << "Working in temporary directory: " << base_path << endl;
  bfs::create_directory(base_path);

  path_t output_path_1 = base_path / "test_config_output.conf";
  cerr << "Writing config_block to: " << output_path_1 << endl;
  write_config_file(orig_config, output_path_1);

  path_t output_path_2 = base_path / "subdir" / "test_config_output.conf";
  cerr << "Writing config_block to: " << output_path_2 << endl;
  write_config_file(orig_config, output_path_2);

  // Read files back in, confirning output is readable and the same as
  // what we should have output.
  std::vector<config_block_sptr> configs;
  configs.push_back(read_config_file(output_path_1));
  configs.push_back(read_config_file(output_path_2));

  BOOST_FOREACH( config_block_sptr config, configs)
  {
    TEST_EQUAL("num params", config->available_values().size(), 7);
    TEST_EQUAL("key-A read",
               config->get_value<config_block_value_t>(keyA),
               valueA);
    TEST_EQUAL("key-B read",
               config->get_value<config_block_value_t>(keyB),
               valueB);
    TEST_EQUAL("subblock key-C read",
               config->get_value<config_block_value_t>(subblock_name + config_block::block_sep + keyC),
               valueC);
    TEST_EQUAL("key-D read",
               config->get_value<config_block_value_t>(keyD),
               valueD);
    TEST_EQUAL("key-E read",
               config->get_value<config_block_value_t>(keyE),
               valueE);
    TEST_EQUAL("key-F read",
               config->get_value<config_block_value_t>(keyF),
               valueF);
    TEST_EQUAL("key-G read",
               config->get_value<config_block_value_t>(keyG),
               valueG);
  }

  cerr << "Cleaning up temp directory: " << base_path << endl;
  bfs::remove_all(base_path);
}

IMPLEMENT_TEST(invalid_directory_write)
{
  using namespace maptk;
  config_block_sptr config = config_block::empty_config("empty");
  config->set_value("foo", "bar");
  EXPECT_EXCEPTION(
      file_write_exception,
      write_config_file(config, data_dir),
      "attempting write on a directory path"
      );
}

IMPLEMENT_TEST(empty_config_write_failure)
{
  using namespace std;
  using namespace maptk;
  namespace bfs = boost::filesystem;

  config_block_sptr config = config_block::empty_config("empty");
  path_t output_file = bfs::temp_directory_path() / bfs::unique_path();
  EXPECT_EXCEPTION(
      file_write_exception,
      write_config_file(config, output_file),
      "attempted write of a config with nothing in it"
      );
  // If the test failed, clean-up the file created.
  if(bfs::is_regular_file(output_file))
  {
    cerr << "Test failed and output file created. Removing." << endl;
    bfs::remove(output_file);
  }
}
