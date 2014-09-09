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
 * \brief Feature tracker utility
 */

#include <fstream>
#include <string>
#include <vector>

#include <maptk/modules.h>

#include <maptk/core/algo/image_io.h>
#include <maptk/core/algo/convert_image.h>
#include <maptk/core/algo/detect_features.h>
#include <maptk/core/algo/estimate_homography.h>
#include <maptk/core/algo/extract_descriptors.h>
#include <maptk/core/algo/match_features.h>

#include <maptk/core/config_block.h>
#include <maptk/core/config_block_io.h>
#include <maptk/core/image_container.h>
#include <maptk/core/exceptions.h>
#include <maptk/core/types.h>

#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/make_shared.hpp>

namespace bfs = boost::filesystem;
namespace bpo = boost::program_options;


/// Logging / Debugging helper macros
#ifndef NDEBUG
/// Display a debugging message
# define LOG_DEBUG(msg) \
  std::cerr << "[mss][DEBUG] " << msg << std::endl
/// Execute debug code
# define DEBUG_CODE(code) code
#else
/// Display a debugging message
# define LOG_DEBUG(msg)
/// Execute debug code
# define DEBUG_CODE(code)
#endif

/// Display an informational message
#define LOG_INFO(msg) \
  std::cerr << "[mss] [INFO] " << msg << std::endl

/// Display a warning message
#define LOG_WARNING(msg) \
  std::cerr << "[mss] [WARN] " << msg << std::endl
#define LOG_WARN(msg) \
  LOG_WARNING(msg)

/// Display an error message
#define LOG_ERROR(msg) \
  std::cerr << "[mss][ERROR] " << msg << std::endl


static void print_usage(std::string const &prog_name,
                        bpo::options_description const &opt_desc,
                        bpo::options_description const &pos_desc)
{
  std::cerr << std::endl
            << "USAGE: " << prog_name << " [OPTS] img1 img2 output_file" << std::endl
            << std::endl;

  std::cerr << "Positional arguments:" << std::endl
            << pos_desc << std::endl;

  std::cerr << "Options:" << std::endl
            << opt_desc << std::endl;
}


// Macro allowing for other macro calls on standard stitcher algorithms.
// Call must be two take two arguments: (algo_type, algo_name)
#define stitcher_algos(call)                       \
  call(image_io,            image_reader);         \
  call(convert_image,       image_converter);      \
  call(detect_features,     feature_detector);     \
  call(extract_descriptors, descriptor_extractor); \
  call(match_features,      feature_matcher);      \
  call(estimate_homography, homog_estimator)


static maptk::config_block_sptr default_config()
{
  maptk::config_block_sptr config = maptk::config_block::empty_config("stitcher_tool");

  // Default algorithm types
  config->set_value("image_reader:type", "vxl");
  config->set_value("image_converter:type", "default");
  config->set_value("descriptor_extractor:type", "ocv");
  config->set_value("descriptor_extractor:ocv:extractor:type", "Feature2D.SURF");
  config->set_value("feature_detector:type", "ocv");
  config->set_value("feature_detector:ocv:detector:type", "Feature2D.SURF");
  config->set_value("feature_matcher:type", "ocv");
  config->set_value("feature_matcher:ocv:matcher:type", "DescriptorMatcher.FlannBasedMatcher");
  config->set_value("homog_estimator:type", "ocv");
  // TODO: homography generation algo thingy

  // expand algo config if any
#define get_default(type, name) \
  maptk::algo::type::get_nested_algo_configuration( #name, config, maptk::algo::type##_sptr() );

  stitcher_algos(get_default);

#undef get_default

  return config;
}


static bool check_config(maptk::config_block_sptr config)
{
  bool config_valid = true;

#define MAPTK_CONFIG_FAIL(msg)            \
  LOG_WARN("Config Check Fail: " << msg); \
  config_valid = false

#define check_algo_config(type, name)                                              \
  if (! maptk::algo::type::check_nested_algo_configuration( #name, config ))       \
  {                                                                                \
    MAPTK_CONFIG_FAIL("Configuration for algorithm " << #name << " was invalid."); \
  }

  stitcher_algos(check_algo_config);

#undef check_algo_config

#undef MAPTK_CONFIG_FAIL

  return config_valid;
}


#define print_config(config) \
  do \
  { \
    BOOST_FOREACH( maptk::config_block_key_t key, config->available_values() ) \
    { \
      std::cerr << "\t" \
           << key << " = " << config->get_value<maptk::config_block_key_t>(key) \
           << std::endl; \
    } \
  } while (false)


static int maptk_main(int argc, char const* argv[])
{
  // register the algorithms in the various modules for dynamic look-up
  maptk::register_modules();

  //
  // define/parse CLI options
  //

  // options
  bpo::options_description opt_desc;
  opt_desc.add_options()
    ("help,h", "output help message and exit")
    ("config,c",
     bpo::value<maptk::path_t>()->value_name("PATH"),
     "Optional custom configuration file for the tool.")
    ("output-config,o",
     bpo::value<maptk::path_t>()->value_name("PATH"),
     "Output a configuration file with default values. This may be seeded with "
     "a configuration file from -c/--config.")
    ("inlier-scale,i",
     bpo::value<double>()->value_name("DOUBLE")->default_value(10.0),
     "Error distance tolerated for matches to be considered inliers during homography estimation.")
    ;
  // input file positional collector
  bpo::options_description opt_desc_pos;
  bpo::positional_options_description pos_opt_desc;
  opt_desc_pos.add_options()
    ("input_img_files",
     bpo::value< std::vector<std::string> >()->value_name("PATH"),
     "2 in-frame-order image files")
    ("output_homog_file",
     bpo::value<std::string>()->value_name("PATH"),
     "File to write generated homography transformation between input frames "
     "to. This ends up including two homographies: An identity associated to "
     "the first frame and then an actual homography describing the "
     "transformation to the second frame.");
  pos_opt_desc.add("input_img_files", 2)
              .add("output_homog_file", 1);
  // option accregation
  bpo::options_description all_opts;
  all_opts.add(opt_desc).add(opt_desc_pos);

  bpo::variables_map vm;
  try
  {
    bpo::store(bpo::command_line_parser(argc, argv)
                    .options(all_opts)
                    .positional(pos_opt_desc)
                    .run(),
               vm);
    bpo::notify(vm);
  }
  catch (bpo::unknown_option const& e)
  {
    LOG_ERROR("Unknown option: " << e.get_option_name());
    print_usage(argv[0], opt_desc, opt_desc_pos);
    return EXIT_FAILURE;
  }
  catch (bpo::error const &e)
  {
    LOG_ERROR("Boost Program Options error: " << e.what());
    print_usage(argv[0], opt_desc, opt_desc_pos);
    return EXIT_FAILURE;
  }

  if(vm.count("help"))
  {
    print_usage(argv[0], opt_desc, opt_desc_pos);
    return EXIT_SUCCESS;
  }

  // Set config to algo chain
  // Get config from algo chain after set
  // Check config validity, store result
  //
  // If -o/--output-config given, output config result and notify of current (in)validity
  // Else error if provided config not valid.

  //
  // Setup algorithms and configuration
  //

  namespace algo = maptk::algo;

  maptk::config_block_sptr config = default_config();

  // Define algorithm variables.
#define define_algo(type, name) \
  algo::type##_sptr name

  stitcher_algos(define_algo);

#undef define_algo

  // If -c/--config given, read in confg file, merge onto default just generated
  if(vm.count("config"))
  {
    config->merge_config(maptk::read_config_file(vm["config"].as<maptk::path_t>()));
  }

  // Set current configuration to algorithms and extract refined configuration.
#define sa(type, name)                                                       \
  algo::type::set_nested_algo_configuration( #name, config, name ); \
  algo::type::get_nested_algo_configuration( #name, config, name )

  stitcher_algos(sa);

#undef sa

  // Check that current configuration is valid.
  bool valid_config = check_config(config);

  if (vm.count("output-config"))
  {
    //std::cerr << "[DEBUG] Given config output target: " << vm["output-config"].as<maptk::path_t>() << std::endl;
    write_config_file(config, vm["output-config"].as<maptk::path_t>());
    if(valid_config)
    {
      std::cerr << "INFO: Configuration file contained valid parameters and may be used for running" << std::endl;
    }
    else
    {
      std::cerr << "WARNING: Configuration deemed not valid." << std::endl;
    }
    return EXIT_SUCCESS;
  }
  else if(!valid_config)
  {
    std::cerr << "ERROR: Configuration not valid." << std::endl;
    return EXIT_FAILURE;
  }

  // Check for correct input image file-path arguments
  if (!vm.count("input_img_files"))
  {
    LOG_ERROR("No input image files were given.");
    return EXIT_FAILURE;
  }
  else if (vm["input_img_files"].as<std::vector<std::string> >().size() != 2)
  {
    LOG_ERROR("Require 2 input images. "
              << vm["input_img_files"].as<std::vector<std::string> >().size()
              << " given.");
    return EXIT_FAILURE;
  }

  // Check for output file argument for generated homography
  if (!vm.count("output_homog_file"))
  {
    LOG_ERROR("No output homography file path specified!");
    return EXIT_FAILURE;
  }
  std::ofstream homog_output_stream(vm["output_homog_file"].as<std::string>().c_str());
  if (!homog_output_stream)
  {
    LOG_ERROR("Could not open output homog file: " << vm["output_homog_file"].as<std::string>());
    return EXIT_FAILURE;
  }

  LOG_DEBUG("Loading images");
  std::vector<std::string> input_img_files(vm["input_img_files"].as< std::vector<std::string> >());
  maptk::image_container_sptr i1_image, i2_image;
  try
  {
    i1_image = image_converter->convert(image_reader->load(input_img_files[0]));
    i2_image = image_converter->convert(image_reader->load(input_img_files[1]));
  }
  catch (maptk::path_not_exists const &e)
  {
    LOG_ERROR(e.what());
    return EXIT_FAILURE;
  }
  catch (maptk::path_not_a_file const &e)
  {
    LOG_ERROR(e.what());
    return EXIT_FAILURE;
  }

  LOG_DEBUG("Generating features over input frames");
  maptk::feature_set_sptr i1_features = feature_detector->detect(i1_image),
                          i2_features = feature_detector->detect(i2_image);
  LOG_DEBUG("Generating descriptors over input frames");
  maptk::descriptor_set_sptr i1_descriptors = descriptor_extractor->extract(i1_image, i1_features),
                             i2_descriptors = descriptor_extractor->extract(i2_image, i2_features);

  LOG_DEBUG("Matching features");
  maptk::match_set_sptr matches = feature_matcher->match(i1_features, i1_descriptors,
                                                         i2_features, i2_descriptors);

  LOG_DEBUG("Estimating homography");
  std::vector<bool> inliers;
  maptk::homography homog = homog_estimator->estimate(i1_features, i2_features,
                                                      matches, inliers);

  LOG_DEBUG("Writing homography file");
  maptk::homography identity;
  identity.set_identity();
  homog_output_stream << identity << "\n" << homog;
  homog_output_stream.close();

  return EXIT_SUCCESS;
}


int main(int argc, char const* argv[])
{
  try
  {
    return maptk_main(argc, argv);
  }
  catch (std::exception const& e)
  {
    std::cerr << "Exception caught: " << e.what() << std::endl;

    return EXIT_FAILURE;
  }
  catch (...)
  {
    std::cerr << "Unknown exception caught" << std::endl;

    return EXIT_FAILURE;
  }
}
