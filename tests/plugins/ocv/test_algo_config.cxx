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


#include <opencv2/core/core.hpp>

#include <vital/exceptions.h>
#include <vital/logger/logger.h>
#include <vital/vital_types.h>
#include <vital/vital_foreach.h>

#include <maptk/plugins/ocv/detect_features.h>
#include <maptk/plugins/ocv/extract_descriptors.h>
#include <maptk/plugins/ocv/match_features.h>
#include <maptk/plugins/ocv/register_algorithms.h>

#include <opencv2/features2d/features2d.hpp>

#define TEST_ARGS ()

DECLARE_TEST_MAP();


int
main(int argc, char* argv[])
{
  CHECK_ARGS(1);

  kwiver::maptk::ocv::register_algorithms();

  testname_t const testname = argv[1];

  RUN_TEST(testname);
}


/// Logs configuration entries given a logging function that takes a single std::string argument
#define log_config(lfunc, config) \
  std::stringstream lcss; \
  VITAL_FOREACH( kwiver::vital::config_block_key_t key, config->available_values() ) \
  { \
    lcss.str(""); \
    lcss << "\t// " << config->get_description(key) << "\n" \
         << "\t" << key << " = " \
                 << config->get_value<kwiver::vital::config_block_key_t>(key); \
    lfunc( lcss.str() ); \
  }

using namespace std;
using namespace kwiver::vital;
using namespace kwiver::maptk;


/// Return vector of all OCV based algo impls
std::vector<algorithm_sptr> get_ocv_algos()
{
  // Make vector of impls
  std::vector<algorithm_sptr> algo_impls;

#define ADD( l, a ) l.push_back( a )
#ifndef MAPTK_HAS_OPENCV_VER_3
#define ADD_OCV2( l, a ) ADD( l, a )
#define ADD_OCV3( l, a )
#else
#define ADD_OCV2( l, a )
#define ADD_OCV3( l, a ) ADD( l, a )
#endif

  ADD_OCV3( algo_impls, algo::detect_features::create( "ocv_AGAST" ) );
  ADD     ( algo_impls, algo::detect_features::create( "ocv_BRISK" ) );
  ADD     ( algo_impls, algo::detect_features::create( "ocv_FAST" ) );
  ADD     ( algo_impls, algo::detect_features::create( "ocv_GFTT" ) );
  ADD     ( algo_impls, algo::detect_features::create( "ocv_MSER" ) );
  ADD     ( algo_impls, algo::detect_features::create( "ocv_simple_blob" ) );
  ADD     ( algo_impls, algo::detect_features::create( "ocv_ORB" ) );

  ADD_OCV2( algo_impls, algo::extract_descriptors::create( "ocv_BRIEF" ) );
  ADD     ( algo_impls, algo::extract_descriptors::create( "ocv_BRISK" ) );
  ADD_OCV2( algo_impls, algo::extract_descriptors::create( "ocv_FREAK" ) );
  ADD     ( algo_impls, algo::extract_descriptors::create( "ocv_ORB" ) );

  ADD     ( algo_impls, algo::match_features::create( "ocv_brute_force" ) );
  ADD     ( algo_impls, algo::match_features::create( "ocv_flann_based" ) );

  // Add other/new OCV algorithms here

#undef ADD
#undef ADD_OCV2
#undef ADD_OCV3

  return algo_impls;
}


/**
 * Test that we can get, set and check the configurations for OCV feature
 * detector implementations.
 */
IMPLEMENT_TEST(ocv_algo_config_defaults)
{
  logger_handle_t log = get_logger(
      "maptk.test.plugins.ocv.ocv_algo_config_defaults"
  );

  VITAL_FOREACH( algorithm_sptr a, get_ocv_algos() )
  {
    LOG_INFO(log, "Testing configuration for algorithm instance @" << a.get() );
    LOG_INFO(log, "-- Algorithm info: " << a->type_name() << "::"
                  << a->impl_name() );
    kwiver::vital::config_block_sptr c = a->get_configuration();
    LOG_INFO(log, "-- default config:");
    log_config(log->log_info , c);

    // Checking and setting the config of algo. Default should always be valid
    // thus passing check.
    LOG_INFO(log, "-- checking default config");
    TEST_EQUAL(
        a->type_name() + "::" + a->impl_name() + "_check_config_pre_set",
        a->check_configuration( c ),
        true
    );

    LOG_INFO(log, "-- Setting default config and checking again");
    a->set_configuration( c );
    TEST_EQUAL(
        a->type_name() + "::" + a->impl_name() + "_check_config_post_set",
        a->check_configuration( c ),
        true
    );
  }

}


/**
 * Test that setting and checking and empty configuration block is a valid
 * operation.
 */
IMPLEMENT_TEST(ocv_algo_empty_config)
{
  logger_handle_t log = get_logger(
      "maptk.test.plugins.ocv.algo_empty_config"
  );

  VITAL_FOREACH( algorithm_sptr a, get_ocv_algos() )
  {
    // Checking an empty config. Since there is literally nothing in the config,
    // we should pass here, as the default configuration should be used which
    // should pass (see test "ocv_algo_config_defaults")
    LOG_INFO(log, "Checking empty config for algorithm instance @" << a.get() );
    LOG_INFO(log, "-- Algorithm info: " << a->type_name() << "::"
                  << a->impl_name() );
    kwiver::vital::config_block_sptr
        empty_conf = kwiver::vital::config_block::empty_config();
    TEST_EQUAL("empty config check test",
               a->check_configuration(empty_conf),
               true);

    // Should be able to set an empty config as defaults should take over.
    LOG_INFO(log, "-- setting empty config");
    a->set_configuration(empty_conf);

    // This should also pass as we take an empty type as a "use the default"
    // message
    TEST_EQUAL("post-set config check",
               a->check_configuration(a->get_configuration()),
               true);
  }
}
