/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include<iostream>
#include<fstream>
#include<exception>
#include<string>
#include<vector>

#include <maptk/modules.h>

#include <maptk/core/track_set_io.h>
#include <maptk/core/algo/image_io.h>
#include <maptk/core/algo/detect_features.h>
#include <maptk/core/algo/extract_descriptors.h>
#include <maptk/core/algo/match_features.h>
#include <maptk/core/algo/match_features_homography.h>
#include <maptk/core/algo/track_features.h>
#include <maptk/core/config_block.h>
#include <maptk/core/config_block_io.h>
#include <maptk/core/types.h>

#include <boost/foreach.hpp>

#include <boost/filesystem.hpp>
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/value_semantic.hpp>
#include <boost/program_options/variables_map.hpp>


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

  maptk::algo::track_features::get_nested_algo_configuration("feature_tracker", config, maptk::algo::track_features_sptr());
  maptk::algo::image_io::get_nested_algo_configuration("image_reader", config, maptk::algo::image_io_sptr());

  return config;
}


static bool check_config(maptk::config_block_sptr config)
{
  return (
         config->has_value("image_list_file") && bfs::exists(maptk::path_t(config->get_value<std::string>("image_list_file")))
      && config->has_value("output_tracks_file")
      && maptk::algo::track_features::check_nested_algo_configuration("feature_tracker", config)
      && maptk::algo::image_io::check_nested_algo_configuration("image_reader", config)
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
  // register the algorithms in the various modules for dynamic look-up
  maptk::register_modules();

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

  // Set up top level configuration w/ defaults where applicable.
  maptk::config_block_sptr config = default_config();
  algo::track_features_sptr feature_tracker;
  algo::image_io_sptr image_reader;

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

  // Creating input image list
  std::vector<std::string> files;
  for (std::string line; std::getline(ifs,line); )
  {
    files.push_back(line);
  }

  // verify that we can open the output file for writing
  // so that we don't find a problem only after spending
  // hours of computation time.
  std::ofstream ofs(output_tracks_file.c_str());
  if (!ofs)
  {
    std::cerr << "Error: Could not open track file for writing: \""
              <<output_tracks_file<<"\""<<std::endl;
    return EXIT_FAILURE;
  }

  // Track features on each frame sequentially
  maptk::track_set_sptr tracks;
  for(unsigned i=0; i<files.size(); ++i)
  {
    std::cout << "processing frame "<<i<<": "<<files[i]<<std::endl;
    maptk::image_container_sptr img = image_reader->load(files[i]);
    tracks = feature_tracker->track(tracks, i, img);
  }

  // release the output file so that the function below can write to it.
  ofs.close();
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
