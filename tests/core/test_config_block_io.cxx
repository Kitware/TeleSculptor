/*ckwg +5
 * Copyright 2011-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include <test_common.h>

#include <string>
#include <iostream>

#include <maptk/core/config_block_io.h>
#include <maptk/core/exceptions.h>
#include <maptk/core/types.h>

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
  maptk::config_block_t config = maptk::read_config_file(data_dir / "test_config-valid_file.txt");

  using std::cerr;
  using std::endl;
  cerr << "Available keys in the config_block:" << endl;
  BOOST_FOREACH(maptk::config_block_key_t key, config->available_values())
  {
    cerr << "\t\"" << key << "\" := \"" << config->get_value<std::string>(key) << "\"" << endl;
  }

  TEST_EQUAL("num config params",
             config->available_values().size(),
             7);
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
  TEST_EQUAL("tabbed:value read",
             config->get_value<std::string>("tabbed:value"),
             "should be valid");

  // extract sub-block, see that value access maintained
  maptk::config_block_t foo_subblock = config->subblock_view("foo");
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
  maptk::config_block_t config = maptk::read_config_file(data_dir / "test_config-valid_file.txt",
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
             7);
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
  maptk::config_block_t config = maptk::read_config_file(data_dir / "test_config-comments.txt");

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

  config_block_t orig_config = config_block::empty_config("simple_test");
  orig_config->set_value("test_key_1", "test_value_a");
  orig_config->set_value("test_key_2", "test_value_b");
  config_block_t subblock = orig_config->subblock_view("subblock");
  subblock->set_value("foobar", "baz");

  cerr << "ConfigBlock for writing:" << endl;
  BOOST_FOREACH( config_block_key_t key, orig_config->available_values() )
  {
    cerr << "\t" << key << " :: " << orig_config->get_value<config_block_value_t>(key) << endl;
  }

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
  std::vector<config_block_t> configs;
  configs.push_back(read_config_file(output_path_1));
  configs.push_back(read_config_file(output_path_2));

  BOOST_FOREACH( config_block_t config, configs)
  {
    TEST_EQUAL("num params", config->available_values().size(), 3);
    TEST_EQUAL("test_key_1 read", config->get_value<config_block_value_t>("test_key_1"),
                                  "test_value_a");
    TEST_EQUAL("test_key_2 read", config->get_value<config_block_value_t>("test_key_2"),
                                  "test_value_b");
    TEST_EQUAL("subblock:foobar read", config->get_value<config_block_value_t>("subblock:foobar"),
                                       "baz");
  }

  cerr << "Cleaning up temp directory" << endl;
  bfs::remove_all(base_path);
}

IMPLEMENT_TEST(invalid_directory_write)
{
  using namespace maptk;
  config_block_t config = config_block::empty_config("empty");
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

  config_block_t config = config_block::empty_config("empty");
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
