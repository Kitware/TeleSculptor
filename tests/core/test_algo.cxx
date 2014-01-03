/*ckwg +5
 * Copyright 2011-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include <test_common.h>

#include <iostream>

#include <maptk/core/algo/register.h>

#include <maptk/core/algo/track_features.h>
#include <maptk/core/config_block.h>
#include <maptk/core/exceptions/algorithm.h>
#include <maptk/core/types.h>

#include <boost/foreach.hpp>

#define TEST_ARGS ()
DECLARE_TEST_MAP();

int main(int argc, char* argv[])
{
  // just the test name is expected
  CHECK_ARGS(1);

  testname_t const testname = argv[1];

  RUN_TEST(testname);
}

IMPLEMENT_TEST(track_features_before_configuration)
{
  using namespace maptk;
  using namespace maptk::algo;

  // register core algorithms
  maptk::algo::register_algorithms();

  track_features_sptr track_features_impl = track_features::create("simple");
  config_block_sptr tf_config = track_features_impl->get_configuration();

  config_block_sptr nested_alg = config_block::empty_config("nested_alg");
  nested_alg->set_value("sub_parameter", "some value");
  tf_config->subblock_view(nested_alg->get_name())->merge_config(nested_alg);

  using namespace std;
  cerr << "Contents of config_block:" << endl;
  BOOST_FOREACH( config_block_key_t key, tf_config->available_values() )
  {
    cerr << "\t"
         << key << " := " << tf_config->get_value<config_block_value_t>(key)
         << endl;
  };

  EXPECT_EXCEPTION(
      algorithm_configuration_exception,
      track_features_impl->configure(tf_config),
      "configuring track_features algorithm with a known invalid config"
      );
}
