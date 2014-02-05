/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief Tests involving OCV nested algorithms and their parameter
 *        interactions with config_block objects.
 */

#include <test_common.h>

#include <iostream>

#include <boost/foreach.hpp>

#include <maptk/core/exceptions.h>
#include <maptk/core/types.h>

#include <maptk/ocv/detect_features.h>
#include <maptk/ocv/extract_descriptors.h>
#include <maptk/ocv/match_features.h>
#include <maptk/ocv/register.h>

#define TEST_ARGS ()

DECLARE_TEST_MAP();

int
main(int argc, char* argv[])
{
  CHECK_ARGS(1);

  maptk::ocv::register_algorithms();

  testname_t const testname = argv[1];

  RUN_TEST(testname);
}

#define print_config(config) \
  BOOST_FOREACH( config_block_key_t key, config->available_values() ) \
  { \
    std::cerr << "\t" \
         << key << " = " << config->get_value<config_block_key_t>(key) \
         << std::endl; \
  }


IMPLEMENT_TEST(detect_features_args)
{
  using namespace std;
  using namespace maptk;

  algo::detect_features_sptr df = algo::detect_features::create("ocv");
  config_block_sptr config = df->get_configuration();

  cerr << "[test-detect_features_args] First-time get-config:" << endl;
  print_config(config);

  // test checking and setting the config of algo
  cerr << "[test-detect_features_args] Checking configuration" << endl;
  TEST_EQUAL("config check",
             df->check_configuration(config),
             true);
  cerr << "[test-detect_features_args] Setting configuration" << endl;
  df->set_configuration(config);

  // an empty config should fail the check test
  config_block_sptr empty_conf = config_block::empty_config();
  TEST_EQUAL("empty config check test",
             df->check_configuration(empty_conf),
             false);

  EXPECT_EXCEPTION(
    config_block_exception,
    df->set_configuration(empty_conf),
    "setting empty config");
}


IMPLEMENT_TEST(extract_descriptors_args)
{
  using namespace std;
  using namespace maptk;

  algo::extract_descriptors_sptr ed = algo::extract_descriptors::create("ocv");
  config_block_sptr config = ed->get_configuration();

  cerr << "[test-extract_descriptors_args] First-time get-config:" << endl;
  print_config(config);

  // test checking and setting the config of algo
  cerr << "[test-extract_descriptors_args] Checking configuration" << endl;
  TEST_EQUAL("config check",
             ed->check_configuration(config),
             true);
  cerr << "[test-extract_descriptors_args] Setting configuration" << endl;
  ed->set_configuration(config);

  // an empty config should fail the check test
  config_block_sptr empty_conf = config_block::empty_config();
  TEST_EQUAL("empty config check test",
             ed->check_configuration(empty_conf),
             false);

  EXPECT_EXCEPTION(
    config_block_exception,
    ed->set_configuration(empty_conf),
    "setting empty config");
}


IMPLEMENT_TEST(match_features_args)
{
  using namespace std;
  using namespace maptk;

  algo::match_features_sptr mf = algo::match_features::create("ocv");
  config_block_sptr config = mf->get_configuration();

  cerr << "[test-match_features_args] First-time get-config:" << endl;
  print_config(config);

  // test checking and setting the config of algo
  cerr << "[test-match_features_args] Checking configuration" << endl;
  TEST_EQUAL("config check",
             mf->check_configuration(config),
             true);
  cerr << "[test-match_features_args] Setting configuration" << endl;
  mf->set_configuration(config);

  // an empty config should fail the check test
  /*
  config_block_sptr empty_conf = config_block::empty_config();
  TEST_EQUAL("empty config check test",
             mf->check_configuration(empty_conf),
             false);

  EXPECT_EXCEPTION(
    config_block_exception,
    mf->set_configuration(empty_conf),
    "setting empty config");
  */
}
