/*ckwg +29
 * Copyright 2013-2015 by Kitware, Inc.
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
 * \brief Tests involving OCV nested algorithms and their parameter
 *        interactions with kwiver::vital::config_block objects.
 */

#include <test_common.h>

#include <iostream>

#include <boost/foreach.hpp>

#include <opencv2/core/core.hpp>

#include <vital/exceptions.h>
#include <vital/vital_types.h>

#include <maptk/plugins/ocv/detect_features.h>
#include <maptk/plugins/ocv/extract_descriptors.h>
#include <maptk/plugins/ocv/match_features.h>
#include <maptk/plugins/ocv/ocv_algo_tools.h>
#include <maptk/plugins/ocv/register_algorithms.h>

#include <opencv2/features2d/features2d.hpp>

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
  BOOST_FOREACH( kwiver::vital::config_block_key_t key, config->available_values() ) \
  { \
    std::cerr << "\t" \
         << key << " = " << config->get_value<kwiver::vital::config_block_key_t>(key) \
         << std::endl; \
  }


using namespace std;
using namespace kwiver::vital;
using namespace maptk;

IMPLEMENT_TEST(detect_features_opencv)
{
  // Tests our ability to construct an OpenCV algorithm, specifically GridORB,
  // and access the underlying nested algorithm.
  //
  // NOTE: cv::Algorithm::getAlgorithm() returns a cv::Algorithm, and thats it.

  cerr << "Creating algo in a variety of ways" << endl;
  cv::Ptr<cv::FeatureDetector> fd_fd = cv::FeatureDetector::create("GridORB");
  cv::Ptr<cv::Algorithm>     algo_fd = cv::FeatureDetector::create("GridORB");

  cerr << "- fd-fd           constructed" << endl;
  cv::Ptr<cv::Algorithm> nested1 = fd_fd->getAlgorithm("detector");
  TEST_EQUAL("fd-fd nested algo not empty", nested1.empty(), 0);
  if ( !nested1.empty() )
  {
    cerr << "  - Before extraction..." << endl;
    cerr << "  - After extraction: " << nested1->info()->name() << endl;
  }

  cerr << "- algo-fd         constructed" << endl;
  cv::Ptr<cv::Algorithm> nested2 = algo_fd->getAlgorithm("detector");
  TEST_EQUAL("algo-fd nested algo not empty", nested2.empty(), 0);
  if ( !nested2.empty() )
  {
    cerr << "  - Before extraction..." << endl;
    cerr << "  - After extraction: " << nested2->info()->name() << endl;
  }
}


IMPLEMENT_TEST(detect_features_args_defaults)
{
  algo::detect_features_sptr df = algo::detect_features::create("ocv");

  cerr << "[] Getting default configuration:" << endl;
  kwiver::vital::config_block_sptr config = df->get_configuration();

  cerr << "[] First-time get-config:" << endl;
  print_config(config);

  // test checking and setting the config of algo. Default should always
  // pass check.
  cerr << "[] Checking configuration" << endl;
  TEST_EQUAL("config check",
             df->check_configuration(config),
             true);

  cerr << "[] Re-Setting default configuration" << endl;
  df->set_configuration(config);
}


IMPLEMENT_TEST(detect_features_empty_configs)
{
  algo::detect_features_sptr df = algo::detect_features::create("ocv");

  // Checking an empty config. Since there is literally nothing in the config,
  // we should pass here, allowing the use of default algorithm type and
  // parameters.
  cerr << "[] Checking empty config" << endl;
  kwiver::vital::config_block_sptr empty_conf = kwiver::vital::config_block::empty_config();
  TEST_EQUAL("empty config check test",
             df->check_configuration(empty_conf),
             true);

  // Should be able to set an empty config as defaults should take over.
  cerr << "[] Setting empty config" << endl;
  df->set_configuration(empty_conf);

  // This should also pass as we take an empty type as a "use the default"
  // message
  cerr << "[] Checking config with '' type configuration" << endl;
  empty_conf->set_value("detector:type", "");
  TEST_EQUAL("no-type config check",
             df->check_configuration(empty_conf),
             true);
}


IMPLEMENT_TEST(algo_set_empty_pointer)
{
  //
  cerr << "--- Testing setting with an empty pointer ---" << endl;
  //
  cv::Ptr<cv::Algorithm> algo_ptr;
  kwiver::vital::config_block_sptr new_config = kwiver::vital::config_block::empty_config(),
                    dflt_config = algo::detect_features::create("ocv")->get_configuration();


  TEST_EQUAL("ptr null test",
             algo_ptr.empty(),
             true);
  // using get_configuration on null ptr should return empty config
  cerr << "[] Extracting NULL algorithm's configuration" << endl;
  ocv::get_nested_ocv_algo_configuration("detector", new_config, algo_ptr);
  cerr << "[] New_config state:" << endl;
  print_config(new_config);
  TEST_EQUAL("new_config empty after get",
             new_config->available_values().size(),
             1);
  TEST_EQUAL("new_config empty type value check",
             new_config->get_value<std::string>("detector:type"),
             "");

  cerr << "[] dflt_onfig state:" << endl;
  print_config(dflt_config);
  cerr << "[] Setting algo_ptr using dflt_config generated earlier" << endl;
  ocv::set_nested_ocv_algo_configuration("detector", dflt_config, algo_ptr);
  cerr << "[] Post-set algo_ptr name: " << algo_ptr->info()->name() << endl;

  // new ptr should now have a value
  TEST_EQUAL("algo_ptr now initialized test",
             algo_ptr.empty(),
             false);
  cerr << "[] New_ptr should now generate a valid config through probe" << endl;
  ocv::get_nested_ocv_algo_configuration("detector", new_config, algo_ptr);
  cerr << "[] Generated config from setting raw ptr:" << endl;
  print_config(new_config);
}


IMPLEMENT_TEST(detect_features_subclass_type_label)
{
  //
  cerr << "--- Test resetting algorithm type :: sub-class custom label ---" << endl;
  //
  algo::detect_features_sptr df = algo::detect_features::create("ocv");
  cerr << "[] Creating empty config except for a detector type" << endl;
  kwiver::vital::config_block_sptr new_config = kwiver::vital::config_block::empty_config();
  new_config->set_value("detector:type", "ORB");
  cerr << "[] pre-set configuration:" << endl;
  print_config(new_config);
  cerr << "[] config check result: " << df->check_configuration(new_config) << endl;
  TEST_EQUAL("config check test", df->check_configuration(new_config), true);
  df->set_configuration(new_config);
  new_config = df->get_configuration();
  cerr << "[] post-set configuration:" << endl;
  print_config(new_config);
}


IMPLEMENT_TEST(detect_features_general_type_label)
{
  //
  cerr << "--- Test resetting algorithm type :: general label ---" << endl;
  //
  algo::detect_features_sptr df = algo::detect_features::create("ocv");
  cerr << "[] Creating empty config except for a detector type" << endl;
  kwiver::vital::config_block_sptr new_config = kwiver::vital::config_block::empty_config();
  new_config->set_value("detector:type", "Feature2D.ORB");
  cerr << "[] pre-set configuration:" << endl;
  print_config(new_config);
  cerr << "[] config check result: " << df->check_configuration(new_config) << endl;
  df->set_configuration(new_config);
  new_config = df->get_configuration();
  cerr << "[] post-set configuration:" << endl;
  print_config(new_config);
}


IMPLEMENT_TEST(extract_descriptors_args_defaults)
{
  algo::extract_descriptors_sptr ed = algo::extract_descriptors::create("ocv");

  cerr << "[] Getting default configuration:" << endl;
  kwiver::vital::config_block_sptr config = ed->get_configuration();

  cerr << "[] First-time get-config:" << endl;
  print_config(config);

  // test checking and setting the config of algo. Default should always
  // pass check.
  cerr << "[] Checking configuration" << endl;
  TEST_EQUAL("config check",
             ed->check_configuration(config),
             true);

  cerr << "[] Re-Setting default configuration" << endl;
  ed->set_configuration(config);
}


IMPLEMENT_TEST(extract_descriptors_empty_configs)
{
  algo::extract_descriptors_sptr ed = algo::extract_descriptors::create("ocv");

  // Checking an empty config. Since there is literally nothing in the config,
  // we should pass here, allowing the use of default algorithm type and
  // parameters.
  cerr << "[] Checking empty config" << endl;
  kwiver::vital::config_block_sptr empty_conf = kwiver::vital::config_block::empty_config();
  TEST_EQUAL("empty config check test",
             ed->check_configuration(empty_conf),
             true);

  // Should be able to set an empty config as defaults should take over.
  cerr << "[] Setting empty config" << endl;
  ed->set_configuration(empty_conf);

  // This should also pass as we take an empty type as a "use the default"
  // message
  cerr << "[] Checking config with '' type configuration" << endl;
  empty_conf->set_value("extractor:type", "");
  TEST_EQUAL("no-type config check",
             ed->check_configuration(empty_conf),
             true);
}


IMPLEMENT_TEST(match_features_args_defaults)
{
  algo::match_features_sptr mf = algo::match_features::create("ocv");

  cerr << "[] Getting default configuration:" << endl;
  kwiver::vital::config_block_sptr config = mf->get_configuration();

  cerr << "[] First-time get-config:" << endl;
  print_config(config);

  // test checking and setting the config of algo. Default should always
  // pass check.
  cerr << "[] Checking configuration" << endl;
  TEST_EQUAL("config check",
             mf->check_configuration(config),
             true);

  cerr << "[] Re-Setting default configuration" << endl;
  mf->set_configuration(config);
}


IMPLEMENT_TEST(match_features_empty_configs)
{
  algo::match_features_sptr mf = algo::match_features::create("ocv");

  // Checking an empty config. Since there is literally nothing in the config,
  // we should pass here, allowing the use of default algorithm type and
  // parameters.
  cerr << "[] Checking empty config" << endl;
  kwiver::vital::config_block_sptr empty_conf = kwiver::vital::config_block::empty_config();
  TEST_EQUAL("empty config check test",
             mf->check_configuration(empty_conf),
             true);

  // Should be able to set an empty config as defaults should take over.
  cerr << "[] Setting empty config" << endl;
  mf->set_configuration(empty_conf);

  // This should also pass as we take an empty type as a "use the default"
  // message
  cerr << "[] Checking config with '' type configuration" << endl;
  empty_conf->set_value("matcher:type", "");
  TEST_EQUAL("no-type config check",
             mf->check_configuration(empty_conf),
             true);
}
