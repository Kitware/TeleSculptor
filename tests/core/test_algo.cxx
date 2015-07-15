/*ckwg +29
 * Copyright 2011-2015 by Kitware, Inc.
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
 * \brief Basic algorithm configuration tests
 */

#include <test_common.h>

#include <iostream>

#include <vital/config/config_block.h>

#include <vital/exceptions/algorithm.h>
#include <vital/vital_types.h>
#include <vital/algorithm_plugin_manager.h>

#include <maptk/algo/track_features.h>
#include <maptk/algo/match_features.h>

#include <boost/foreach.hpp>

using namespace kwiver::vital;

#define TEST_ARGS ()
DECLARE_TEST_MAP();

int main(int argc, char* argv[])
{
  // just the test name is expected
  CHECK_ARGS(1);

  testname_t const testname = argv[1];

  RUN_TEST(testname);
}

#define print_config(config) \
  BOOST_FOREACH( kwiver::vital::config_block_key_t key, config->available_values() ) \
  { \
    cerr << "\t" \
         << key << " = " << config->get_value<kwiver::vital::config_block_key_t>(key) \
         << endl; \
  }

IMPLEMENT_TEST(registered_names)
{
  using namespace maptk;
  using namespace std;
  // register algorithms from plugins
  kwiver::vital::algorithm_plugin_manager::instance().register_plugins();

  cout << "registered algorithms (type_name:impl_name)\n";
  BOOST_FOREACH(const string& name, kwiver::vital::algorithm::registered_names())
  {
    cout << "  " << name << endl;
  }

  vector<string> mf_names = kwiver::vital::algorithm::registered_names("match_features");
  cout << "registered \"match_features\" implementations\n";
  BOOST_FOREACH(const string& name, mf_names)
  {
    cout << "  " << name << endl;
  }
}

IMPLEMENT_TEST(create_from_name)
{
  using namespace maptk;
  using namespace std;
  // register algorithms from plugins
  kwiver::vital::algorithm_plugin_manager::instance().register_plugins();

  kwiver::vital::algorithm_sptr empty = kwiver::vital::algorithm::create("", "");
  TEST_EQUAL("create empty algorithm", empty, 0);

  empty = kwiver::vital::algorithm::create("match_features", "not_a_real_impl");
  TEST_EQUAL("create invalid algorithm", empty, 0);

  TEST_EQUAL("has algorithm type",
             kwiver::vital::algorithm::has_type_name("match_features"), true);
  TEST_EQUAL("does not have algorithm type",
             kwiver::vital::algorithm::has_type_name("not_a_real_type"), false);
  TEST_EQUAL("has algorithm impl",
             kwiver::vital::algorithm::has_impl_name("match_features",
                                            "homography_guided"), true);
  TEST_EQUAL("does not have algorithm impl",
             kwiver::vital::algorithm::has_impl_name("match_features",
                                            "not_a_real_impl"), false);
  TEST_EQUAL("invalid type does not have algorithm impl",
             kwiver::vital::algorithm::has_impl_name("not_a_real_type",
                                            "homography_guided"), false);

  kwiver::vital::algorithm_sptr valid = kwiver::vital::algorithm::create("match_features", "homography_guided");
  TEST_EQUAL("create valid algorithm", !valid, false);

  algo::match_features_sptr mf = boost::dynamic_pointer_cast<algo::match_features>(valid);
  TEST_EQUAL("create correct type", !mf, false);

  TEST_EQUAL("create correct impl", mf->impl_name(), "homography_guided");

}

IMPLEMENT_TEST(track_features_before_configuration)
{
  // register algorithms from plugins
  kwiver::vital::algorithm_plugin_manager::instance().register_plugins();

  using namespace std;
  using namespace maptk::algo;

  track_features_sptr track_features_impl = track_features::create("core");

  cerr << "Contents of kwiver::vital::config_block BEFORE attempted configuration:" << endl;
  kwiver::vital::config_block_sptr tf_config = track_features_impl->get_configuration();
  print_config(tf_config);

  cerr << "Setting mf algo impl" << endl;
  tf_config->set_value("feature_matcher:type", "homography_guided");

  cerr << "Contents of kwiver::vital::config_block after cb set:" << endl;
  print_config(tf_config);

  cerr << "Setting modified config to tf algorithm" << endl;
  track_features_impl->set_configuration(tf_config);

  cerr << "algo's config after set:" << endl;
  tf_config = track_features_impl->get_configuration();
  print_config(tf_config);

  cerr << "Setting mf's mf algo type (in config)" << endl;
  tf_config->set_value("feature_matcher:homography_guided:feature_matcher:type", "homography_guided");

  cerr << "Contents of kwiver::vital::config_block after set:" << endl;
  print_config(tf_config);

  track_features_impl->set_configuration(tf_config);

  cerr << "algo's config after second algo set:" << endl;
  tf_config = track_features_impl->get_configuration();
  print_config(tf_config);

  cerr << "One more level for good measure" << endl;
  tf_config->set_value("feature_matcher:homography_guided:feature_matcher:homography_guided:feature_matcher:type", "homography_guided");

  cerr << "Contents of cb after set:" << endl;
  print_config(tf_config);

  track_features_impl->set_configuration(tf_config);

  cerr << "algo's config after third algo set" << endl;
  tf_config = track_features_impl->get_configuration();
  print_config(tf_config);

  cerr << "One more level for good measure" << endl;
  tf_config->set_value("feature_matcher:homography_guided:feature_matcher:homography_guided:feature_matcher:homography_guided:feature_matcher:type", "homography_guided");

  cerr << "Contents of cb after set:" << endl;
  print_config(tf_config);

  track_features_impl->set_configuration(tf_config);

  cerr << "algo's config after third algo set" << endl;
  tf_config = track_features_impl->get_configuration();
  print_config(tf_config);

}

IMPLEMENT_TEST(track_features_check_config)
{
  // register core algorithms
  kwiver::vital::algorithm_plugin_manager::instance().register_plugins();

  using namespace maptk::algo;
  using namespace std;

  track_features_sptr tf_impl = track_features::create("core");

  // Checking that exception is thrown when trying to configure with no config
  // parameters.
  kwiver::vital::config_block_sptr config = kwiver::vital::config_block::empty_config("track_features_check_config");
  TEST_EQUAL("empty config check", tf_impl->check_configuration(config), false);

  // Checking that default impl switch value is invalid (base default is
  // nothing).
  config = tf_impl->get_configuration();
  cerr << "Default config:" << endl;
  print_config(config);
  TEST_EQUAL("default config check", tf_impl->check_configuration(config), false);

  // Adding valid implementation name for match_features algo, but should
  // still fail as the underlying match_features impl wants another nested
  // algo specification.
  config->set_value("feature_matcher:type", "homography_guided");
  //config->set_value("match_features_algorithm:homography_guided:match_features_algorithm", "homography_guided");
  cerr << "Modified configuration:" << endl;
  print_config(config);
  TEST_EQUAL("modified config check", tf_impl->check_configuration(config), false);

  cerr << "Config from perspective of algo with that that config:" << endl;
  tf_impl->set_configuration(config);
  kwiver::vital::config_block_sptr cb = tf_impl->get_configuration();
  print_config(cb);

  // Checking that, even though there were nested algorithms that weren't set,
  // at least the one that we did set propaged correctly and triggered the
  // sub-config generation.
  TEST_EQUAL(
      "param check 1",
      cb->get_value<std::string>("feature_matcher:type"),
      "homography_guided"
      );
  TEST_EQUAL(
      "param check 2",
      cb->has_value("feature_matcher:homography_guided:feature_matcher1:type"),
      true
      );

}
