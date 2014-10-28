/*ckwg +29
 * Copyright 2014 by Kitware, Inc.
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
 * \brief Image homography estimation utility
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
  std::cerr << "[meh][DEBUG] " << msg << std::endl
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
  std::cerr << "[meh] [INFO] " << msg << std::endl

/// Display a warning message
#define LOG_WARNING(msg) \
  std::cerr << "[meh] [WARN] " << msg << std::endl
#define LOG_WARN(msg) \
  LOG_WARNING(msg)

/// Display an error message
#define LOG_ERROR(msg) \
  std::cerr << "[meh][ERROR] " << msg << std::endl


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


// Shortcut macro for arbitrarilly acting over the tool's algorithm elements.
// ``call`` macro must be two take two arguments: (algo_type, algo_name)
#define tool_algos(call)                       \
  call(image_io,            image_reader);         \
  call(convert_image,       image_converter);      \
  call(detect_features,     feature_detector);     \
  call(extract_descriptors, descriptor_extractor); \
  call(match_features,      feature_matcher);      \
  call(estimate_homography, homog_estimator)


static maptk::config_block_sptr default_config()
{
  maptk::config_block_sptr config = maptk::config_block::empty_config("homography_estimation_tool");

  // Default algorithm types
  config->set_value("image_reader:type", "vxl");

  config->set_value("image_converter:type", "default");

  config->set_value("feature_detector:type", "ocv");
  config->set_value("feature_detector:ocv:detector:type", "Feature2D.SURF");
  config->set_value("feature_detector:ocv:detector:Feature2D.SURF:hessianThreshold", 250);
  config->set_value("feature_detector:ocv:detector:Feature2D.SURF:upright", true);

  config->set_value("descriptor_extractor:type", "ocv");
  config->set_value("descriptor_extractor:ocv:extractor:type", "Feature2D.SURF");
  config->set_value("descriptor_extractor:ocv:extractor:Feature2D.SURF:hessianThreshold", 250);
  config->set_value("descriptor_extractor:ocv:extractor:Feature2D.SURF:upright", true);

  config->set_value("feature_matcher:type", "ocv");

  config->set_value("homog_estimator:type", "vxl");

  // expand algo config from defaults above if any
#define get_default(type, name) \
  maptk::algo::type::get_nested_algo_configuration( #name, config, maptk::algo::type##_sptr() );

  tool_algos(get_default);

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

  tool_algos(check_algo_config);

#undef check_algo_config

#undef MAPTK_CONFIG_FAIL

  return config_valid;
}


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
     "Optional custom configuration file for the tool. Defaults are set such "
     "that this is not required.")
    ("output-config,o",
     bpo::value<maptk::path_t>()->value_name("PATH"),
     "Output a configuration file with default values. This may be seeded "
     "with a configuration file from -c/--config.")
    ("inlier-scale,i",
     bpo::value<double>()->value_name("DOUBLE")->default_value(1.0),
     "Error distance tolerated for matches to be considered inliers during "
     "homography estimation.")
    ("mask-image,m",
     bpo::value< std::string >()->value_name("PATH"),
     "Optional boolean mask image where positive values indicate where "
     "features should be detected. This image *must* be the same size as the "
     "input images.")
    ;
  // input file positional collector
  bpo::options_description opt_desc_pos;
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
  bpo::positional_options_description pos_opt_desc;
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

  tool_algos(define_algo);

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

  tool_algos(sa);

#undef sa

  // Check that current configuration is valid.
  bool valid_config = check_config(config);

  if (vm.count("output-config"))
  {
    write_config_file(config, vm["output-config"].as<maptk::path_t>());
    if(valid_config)
    {
      LOG_INFO("Configuration file contained valid parameters and may be used for running");
    }
    else
    {
      LOG_WARNING("Configuration deemed not valid.");
    }
    return EXIT_SUCCESS;
  }
  else if(!valid_config)
  {
    LOG_ERROR("Configuration not valid.");
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

  LOG_INFO("Loading images...");
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

  // load and convert mask image if one was given
  LOG_DEBUG("Before mask load");
  maptk::image_container_sptr mask;
  if( vm.count("mask-image") )
  {
    mask = image_converter->convert(
      image_reader->load( vm["mask-image"].as< std::string >() )
    );
  }

  // Make sure we can open for writting the given homography file path
  std::string homog_output_path(vm["output_homog_file"].as<std::string>());
  std::ofstream homog_output_stream(homog_output_path.c_str());
  if (!homog_output_stream)
  {
    LOG_ERROR("Could not open output homog file: " << vm["output_homog_file"].as<std::string>());
    return EXIT_FAILURE;
  }

  LOG_INFO("Generating features over input frames...");
  // if no mask was loaded, the value of mask at this point will be the same
  // as the default value (uninitialized sptr)
  maptk::feature_set_sptr i1_features = feature_detector->detect(i1_image, mask),
                          i2_features = feature_detector->detect(i2_image, mask);
  LOG_INFO("Generating descriptors over input frames...");
  maptk::descriptor_set_sptr i1_descriptors = descriptor_extractor->extract(i1_image, i1_features),
                             i2_descriptors = descriptor_extractor->extract(i2_image, i2_features);
  LOG_INFO("-- Img1 features / descriptors: " << i1_descriptors->size());
  LOG_INFO("-- Img2 features / descriptors: " << i2_descriptors->size());

  LOG_INFO("Matching features...");
  // matching from frame 2 to 1 explicitly. see below.
  maptk::match_set_sptr matches = feature_matcher->match(i2_features, i2_descriptors,
                                                         i1_features, i1_descriptors);
  LOG_INFO("-- Number of matches: " << matches->size());

  // Because we computed matches from frames 2 to 1, this homography describes
  // the transformation from image2 space to image1 space, which is what
  // warping tools usually want.
  LOG_INFO("Estimating homography...");
  std::vector<bool> inliers;
  maptk::homography homog = homog_estimator->estimate(i2_features, i1_features,
                                                      matches, inliers);
  // Reporting inlier count
  size_t inlier_count = 0;
  BOOST_FOREACH(bool b, inliers)
  {
    if (b)
      ++inlier_count;
  }
  LOG_INFO("-- Inliers: " << inlier_count << " / " << inliers.size());

  LOG_INFO("Writing homography file...");
  maptk::homography identity;
  identity.set_identity();
  homog_output_stream << identity << std::endl << homog << std::endl;
  homog_output_stream.close();
  LOG_INFO("-- '" << homog_output_path << "' finished writing");

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
    LOG_ERROR("Exception caught: " << e.what());

    return EXIT_FAILURE;
  }
  catch (...)
  {
    LOG_ERROR("Unknown exception caught");

    return EXIT_FAILURE;
  }
}
