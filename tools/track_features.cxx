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

#include <iostream>
#include <fstream>
#include <exception>
#include <string>
#include <vector>

#include <maptk/algo/image_io.h>
#include <maptk/algo/convert_image.h>
#include <maptk/algo/track_features.h>

#include <maptk/config_block.h>
#include <maptk/config_block_io.h>
#include <maptk/exceptions.h>
#include <maptk/plugin_manager.h>
#include <maptk/track_set_io.h>
#include <maptk/types.h>

#include <boost/foreach.hpp>

#include <boost/filesystem.hpp>
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/value_semantic.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/make_shared.hpp>

namespace bfs = boost::filesystem;


static maptk::config_block_sptr default_config()
{
  maptk::config_block_sptr config = maptk::config_block::empty_config("feature_tracker_tool");

  config->set_value("image_list_file", "",
                    "Path an input file containing new-line separated paths "
                    "to sequential image files.");
  config->set_value("output_tracks_file", "",
                    "Path to a file to write output tracks to. If this "
                    "file exists, it will be overwritten.");
  config->set_value("output_homography_file", "",
                    "Optional path to a file to write source-to-reference "
                    "homographies for each frame. Leave blank to disable this "
                    "output.");

  maptk::algo::track_features::get_nested_algo_configuration("feature_tracker", config, maptk::algo::track_features_sptr());
  maptk::algo::image_io::get_nested_algo_configuration("image_reader", config, maptk::algo::image_io_sptr());
  maptk::algo::convert_image::get_nested_algo_configuration("convert_image", config, maptk::algo::convert_image_sptr());
  maptk::algo::compute_ref_homography::get_nested_algo_configuration("output_homography_generator",
                                                                     config, maptk::algo::compute_ref_homography_sptr());
  return config;
}


static bool check_config(maptk::config_block_sptr config)
{
  // A given homography file is invalid if it names a directory, or if its
  // parent path either doesn't exist or names a regular file.
  bool valid_out_homogs_file = true,
       valid_out_homogs_algo = true;
  if ( config->has_value("output_homography_file")
    && config->get_value<std::string>("output_homography_file") != "" )
  {
    maptk::path_t fp = config->get_value<maptk::path_t>("output_homography_file");
    if ( bfs::is_directory(fp) )
    {
      std::cerr << "Error: Given output homography file is a directory! "
                << "(Given: " << fp << ")"
                << std::endl;
      valid_out_homogs_file = false;
    }
    else if ( fp.parent_path() != "" &&
              ( (!bfs::is_directory(fp.parent_path())) || bfs::is_regular_file(fp.parent_path()) ) )
    {
      std::cerr << "Error: Given output homography file does not have a valid "
                << "parent path! (Given: " << fp << ")"
                << std::endl;
      valid_out_homogs_file = false;
    }

    // Check that compute_ref_homography algo is correctly configured
    valid_out_homogs_algo = maptk::algo::compute_ref_homography::check_nested_algo_configuration("output_homography_generator",
                                                                                                 config);
  }

  return (
         config->has_value("image_list_file") && bfs::exists(maptk::path_t(config->get_value<std::string>("image_list_file")))
      && config->has_value("output_tracks_file")
      && maptk::algo::track_features::check_nested_algo_configuration("feature_tracker", config)
      && maptk::algo::image_io::check_nested_algo_configuration("image_reader", config)
      && maptk::algo::convert_image::check_nested_algo_configuration("convert_image", config)
      && valid_out_homogs_file && valid_out_homogs_algo
      );
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
  // register the algorithm implementations
  maptk::plugin_manager::instance().register_plugins();

  // define/parse CLI options
  boost::program_options::options_description opt_desc;
  opt_desc.add_options()
    ("help,h", "output help message and exit")
    ("config,c",
     boost::program_options::value<maptk::path_t>(),
     "Configuration file for the tool.")
    ("output-config,o",
     boost::program_options::value<maptk::path_t>(),
     "Output a configuration.This may be seeded with a configuration file from -c/--config.")
    ;
  boost::program_options::variables_map vm;

  try
  {
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, opt_desc),
                                  vm);
  }
  catch (boost::program_options::unknown_option const& e)
  {
    std::cerr << "Error: unknown option " << e.get_option_name() << std::endl;
    return EXIT_FAILURE;
  }

  boost::program_options::notify(vm);

  if(vm.count("help"))
  {
    std::cerr << opt_desc << std::endl;
    return EXIT_SUCCESS;
  }

  // Set config to algo chain
  // Get config from algo chain after set
  // Check config validity, store result
  //
  // If -o/--output-config given, output config result and notify of current (in)validity
  // Else error if provided config not valid.

  namespace algo = maptk::algo;
  namespace bfs = boost::filesystem;

  // Set up top level configuration w/ defaults where applicable.
  maptk::config_block_sptr config = default_config();
  algo::track_features_sptr feature_tracker;
  algo::image_io_sptr image_reader;
  algo::convert_image_sptr image_converter;
  algo::compute_ref_homography_sptr out_homog_generator;

  // If -c/--config given, read in confg file, merge in with default just generated
  if(vm.count("config"))
  {
    //std::cerr << "[DEBUG] Given config file: " << vm["config"].as<maptk::path_t>() << std::endl;
    config->merge_config(maptk::read_config_file(vm["config"].as<maptk::path_t>()));
  }

  //std::cerr << "[DEBUG] Config BEFORE set:" << std::endl;
  //print_config(config);

  algo::track_features::set_nested_algo_configuration("feature_tracker", config, feature_tracker);
  algo::track_features::get_nested_algo_configuration("feature_tracker", config, feature_tracker);
  algo::image_io::set_nested_algo_configuration("image_reader", config, image_reader);
  algo::image_io::get_nested_algo_configuration("image_reader", config, image_reader);
  algo::convert_image::set_nested_algo_configuration("convert_image", config, image_converter);
  algo::convert_image::get_nested_algo_configuration("convert_image", config, image_converter);
  algo::compute_ref_homography::set_nested_algo_configuration("output_homography_generator", config, out_homog_generator);
  algo::compute_ref_homography::get_nested_algo_configuration("output_homography_generator", config, out_homog_generator);

  //std::cerr << "[DEBUG] Config AFTER set:" << std::endl;
  //print_config(config);

  bool valid_config = check_config(config);

  if(vm.count("output-config"))
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

  // Attempt opening input and output files.
  std::string image_list_file = config->get_value<std::string>("image_list_file");
  std::string output_tracks_file = config->get_value<std::string>("output_tracks_file");

  std::ifstream ifs(image_list_file.c_str());
  if (!ifs)
  {
    std::cerr << "Error: Could not open image list \""<<image_list_file<<"\""<<std::endl;
    return EXIT_FAILURE;
  }
  // Creating input image list, checking file existance
  std::vector<maptk::path_t> files;
  for (std::string line; std::getline(ifs,line); )
  {
    files.push_back(line);
    if (!bfs::exists(files[files.size()-1]))
    {
      throw maptk::path_not_exists(files[files.size()-1]);
    }
  }

  // verify that we can open the output file for writing
  // so that we don't find a problem only after spending
  // hours of computation time.
  std::ofstream ofs(output_tracks_file.c_str());
  if (!ofs)
  {
    std::cerr << "Error: Could not open track file for writing: \""
              << output_tracks_file << "\"" << std::endl;
    return EXIT_FAILURE;
  }
  ofs.close();

  // Create the output homography file stream if specified
  // Validity of file path checked during configuration file validity check.
  std::ofstream homog_ofs;
  if ( config->has_value("output_homography_file") )
  {
    maptk::path_t homog_fp = config->get_value<maptk::path_t>("output_homography_file");
    homog_ofs.open( homog_fp.string().c_str() );
    if ( !homog_ofs )
    {
      std::cerr << "Error: Could not open homography file for writing: "
                << homog_fp
                << std::endl;
      return EXIT_FAILURE;
    }
  }

  // Track features on each frame sequentially
  maptk::track_set_sptr tracks;
  for(unsigned i=0; i<files.size(); ++i)
  {
    std::cout << "processing frame "<<i<<": "<<files[i]<<std::endl;
    maptk::image_container_sptr img = image_reader->load(files[i].string());
    maptk::image_container_sptr converted = image_converter->convert(img);
    tracks = feature_tracker->track(tracks, i, converted);

    // Compute ref homography for current frame with current track set + write to file
    // -> still doesn't take into account a full shotbreak, which would incur a track reset
    if ( homog_ofs.is_open() )
    {
      std::cout << "\twritting homography" << std::endl;
      homog_ofs << *(out_homog_generator->estimate(i, tracks)) << std::endl;
    }
  }

  if ( homog_ofs.is_open() )
  {
    homog_ofs.close();
  }

  // Writing out tracks to file
  maptk::write_track_file(tracks, output_tracks_file);

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
