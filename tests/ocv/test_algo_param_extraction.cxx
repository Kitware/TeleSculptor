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

#include <opencv2/core/core.hpp>

#include <maptk/core/exceptions.h>
#include <maptk/core/types.h>

#include <maptk/ocv/detect_features.h>
#include <maptk/ocv/extract_descriptors.h>
#include <maptk/ocv/match_features.h>
#include <maptk/ocv/ocv_algo_tools.h>
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
  cerr << "[test-detect_features_args] Getting default configuration:" << endl;
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

  // Checking an empty config should fail as its missing a type specification.
  // Usually, a default config is retrieved first, like above, before merging
  // in modifications, so at least the default types would be present.
  cerr << "[test-detect_features_args] Checking empty config (expecting failure)" << endl;
  config_block_sptr empty_conf = config_block::empty_config();
  TEST_EQUAL("empty config check test",
             df->check_configuration(empty_conf),
             false);

  // Should be able to set an empty config as defaults should take over.
  cerr << "[test-detect_features_args] Setting empty config" << endl;
  df->set_configuration(empty_conf);

  // Testing setting with an empty pointer
  cv::Ptr<cv::Algorithm> algo_ptr;
  config_block_sptr new_config = config_block::empty_config();
  TEST_EQUAL("ptr null test",
             algo_ptr.empty(),
             true);
  // using get config on null ptr should return empty config
  ocv::get_nested_ocv_algo_configuration("detector", new_config, algo_ptr);
  cerr << "[] New_onfig state:" << endl;
  print_config(new_config);
  TEST_EQUAL("config empty after get",
             new_config->available_values().size(),
             1);
  TEST_EQUAL("config empty type value check",
             new_config->get_value<std::string>("detector:type"),
             "");
  // setting algo_ptr using default config generated earlier
  cerr << "[] Config state:" << endl;
  print_config(config);
  ocv::set_nested_ocv_algo_configuration("detector", config, algo_ptr);
  cerr << "Now set algo_ptr name: " << algo_ptr->info()->name() << endl
       << "Now set algo_ptr detector algo: " << algo_ptr->get<cv::Algorithm>("detector") << endl
       ;

  // new ptr should now have a value
  TEST_EQUAL("algo_ptr now initialized test",
             algo_ptr.empty(),
             false);
  ocv::get_nested_ocv_algo_configuration("detector", new_config, algo_ptr);
  cerr << "[] Generated config from setting raw ptr:" << endl;
  print_config(new_config);
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

  // Checking an empty config should fail as its missing a type specification.
  // Usually, a default config is retrieved first, like above, before merging
  // in modifications, so at least the default types would be present.
  cerr << "[test-extract_descriptors_args] Checking empty config (expecting failure)" << endl;
  config_block_sptr empty_conf = config_block::empty_config();
  TEST_EQUAL("empty config check test",
             ed->check_configuration(empty_conf),
             false);

  // Should be able to set an empty config as defaults should take over.
  cerr << "[test-extract_descriptors_args] Setting empty config" << endl;
  ed->set_configuration(empty_conf);
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

  // Checking an empty config should fail as its missing a type specification.
  // Usually, a default config is retrieved first, like above, before merging
  // in modifications, so at least the default types would be present.
  cerr << "[test-match_features_args] Checking empty config (expecting failure)" << endl;
  config_block_sptr empty_conf = config_block::empty_config();
  TEST_EQUAL("empty config check test",
             mf->check_configuration(empty_conf),
             false);

  // Should be able to set an empty config as defaults should take over.
  cerr << "[test-match_features_args] Setting empty config" << endl;
  mf->set_configuration(empty_conf);
}
