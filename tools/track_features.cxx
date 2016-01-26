/*ckwg +29
 * Copyright 2013-2016 by Kitware, Inc.
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

#include <maptk/colorize.h>

#include <vital/config/config_block.h>
#include <vital/config/config_block_io.h>
#include <vital/logger/logger.h>
#include <vital/vital_foreach.h>

#include <vital/algorithm_plugin_manager.h>
#include <vital/exceptions.h>
#include <vital/io/track_set_io.h>
#include <vital/vital_types.h>
#include <vital/algo/image_io.h>
#include <vital/algo/convert_image.h>
#include <vital/algo/track_features.h>
#include <vital/algo/compute_ref_homography.h>
#include <kwiversys/SystemTools.hxx>


#include <boost/filesystem.hpp>
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/value_semantic.hpp>
#include <boost/program_options/variables_map.hpp>

namespace bfs = boost::filesystem;
typedef kwiversys::SystemTools ST;

static kwiver::vital::logger_handle_t main_logger( kwiver::vital::get_logger( "track_features_tool" ) );

static kwiver::vital::config_block_sptr default_config()
{
  kwiver::vital::config_block_sptr config = kwiver::vital::config_block::empty_config("feature_tracker_tool");

  config->set_value("image_list_file", "",
                    "Path to an input file containing new-line separated paths "
                    "to sequential image files.");
  config->set_value("mask_list_file", "",
                    "Optional path to an input file containing new-line "
                    "separated paths to mask images. This list should be "
                    "parallel in association to files specified in "
                    "``image_list_file``. Mask image must be the same size as "
                    "the image they are associated with.\n"
                    "\n"
                    "Leave this blank if no image masking is desired.");
  config->set_value("invert_masks", false,
                    "If true, all mask images will be inverted after loading. "
                    "This is useful if mask images read in use positive "
                    "values to indicated masked areas instead of non-masked "
                    "areas.");
  config->set_value("expect_multichannel_masks", false,
                    "A majority of the time, mask images are a single channel, "
                    "however it is feasibly possible that certain "
                    "implementations may use multi-channel masks. If this is "
                    "true we will expect multiple-channel mask images, "
                    "warning when a single-channel mask is provided. If this "
                    "is false we error upon seeing a multi-channel mask "
                    "image.");
  config->set_value("output_tracks_file", "",
                    "Path to a file to write output tracks to. If this "
                    "file exists, it will be overwritten.");
  config->set_value("output_homography_file", "",
                    "Optional path to a file to write source-to-reference "
                    "homographies for each frame. Leave blank to disable this "
                    "output. The output_homography_generator algorithm type "
                    "only needs to be set if this is set.");

  kwiver::vital::algo::track_features::get_nested_algo_configuration("feature_tracker", config,
                                      kwiver::vital::algo::track_features_sptr());
  kwiver::vital::algo::image_io::get_nested_algo_configuration("image_reader", config,
                                      kwiver::vital::algo::image_io_sptr());
  kwiver::vital::algo::convert_image::get_nested_algo_configuration("convert_image", config,
                                      kwiver::vital::algo::convert_image_sptr());
  kwiver::vital::algo::compute_ref_homography::get_nested_algo_configuration("output_homography_generator",
                              config, kwiver::vital::algo::compute_ref_homography_sptr());
  return config;
}


static bool check_config(kwiver::vital::config_block_sptr config)
{
  bool config_valid = true;

#define MAPTK_CONFIG_FAIL(msg) \
  std::cerr << "Config Check Fail: " << msg << std::endl; \
  config_valid = false

  // A given homography file is invalid if it names a directory, or if its
  // parent path either doesn't exist or names a regular file.
  if ( config->has_value("output_homography_file")
    && config->get_value<std::string>("output_homography_file") != "" )
  {
    kwiver::vital::config_path_t fp = config->get_value<kwiver::vital::config_path_t>("output_homography_file");
    if ( kwiversys::SystemTools::FileIsDirectory( fp ) )
    {
      MAPTK_CONFIG_FAIL("Given output homography file is a directory! "
                        << "(Given: " << fp << ")");
    }
    else if ( ST::GetFilenamePath( fp ) != "" &&
              ! ST::FileIsDirectory(ST::GetFilenamePath( fp ) ))
    {
      MAPTK_CONFIG_FAIL("Given output homography file does not have a valid "
                        << "parent path! (Given: " << fp << ")");
    }

    // Check that compute_ref_homography algo is correctly configured
    if( !kwiver::vital::algo::compute_ref_homography
             ::check_nested_algo_configuration("output_homography_generator",
                                               config) )
    {
      MAPTK_CONFIG_FAIL("output_homography_generator configuration check failed");
    }
  }

  if (!config->has_value("image_list_file") ||
      config->get_value<std::string>("image_list_file") == "")
  {
    MAPTK_CONFIG_FAIL("Config needs value image_list_file");
  }
  else
  {
    std::string path = config->get_value<std::string>("image_list_file");
    if (!bfs::exists(kwiver::vital::path_t(path)))
    {
      MAPTK_CONFIG_FAIL("image_list_file path, " << path << ", does not exist");
    }
    else if (!bfs::is_regular_file(kwiver::vital::path_t(path)))
    {
      MAPTK_CONFIG_FAIL("image_list_file path, " << path << ", is not a regular file");
    }
  }

  // If given an mask image list file, check that the file exists and is a file
  if (config->has_value("mask_list_file") && config->get_value<std::string>("mask_list_file") != "" )
  {
    std::string mask_list_file = config->get_value<std::string>("mask_list_file");
    if (mask_list_file != "" && !bfs::is_regular_file( kwiver::vital::path_t(mask_list_file) ))
    {
      MAPTK_CONFIG_FAIL("mask_list_file path, " << mask_list_file << ", does not exist");
    }
  }

  if (!config->has_value("output_tracks_file") ||
      config->get_value<std::string>("output_tracks_file") == "" )
  {
    MAPTK_CONFIG_FAIL("Config needs value output_tracks_file");
  }
  else if (!bfs::is_directory(bfs::absolute(
               config->get_value<kwiver::vital::path_t>("output_tracks_file")).parent_path()))
  {
    MAPTK_CONFIG_FAIL("output_tracks_file is not in a valid directory");
  }

  if (!kwiver::vital::algo::track_features::check_nested_algo_configuration("feature_tracker", config))
  {
    MAPTK_CONFIG_FAIL("feature_tracker configuration check failed");
  }

  if (!kwiver::vital::algo::image_io::check_nested_algo_configuration("image_reader", config))
  {
    MAPTK_CONFIG_FAIL("image_reader configuration check failed");
  }

  if (!kwiver::vital::algo::convert_image::check_nested_algo_configuration("convert_image", config))
  {
    MAPTK_CONFIG_FAIL("convert_image configuration check failed");
  }

#undef MAPTK_CONFIG_FAIL

  return config_valid;
}


static kwiver::vital::image::byte invert_mask_pixel( kwiver::vital::image::byte const &b )
{
  return !b;
}


#define print_config(config) \
  do \
  { \
    VITAL_FOREACH( kwiver::vital::config_block_key_t key, config->available_values() ) \
    { \
      std::cerr << "\t" \
           << key << " = " << config->get_value<kwiver::vital::config_block_key_t>(key) \
           << std::endl; \
    } \
  } while (false)


static int maptk_main(int argc, char const* argv[])
{
  // register the algorithm implementations
  kwiver::vital::algorithm_plugin_manager::instance().register_plugins();

  // define/parse CLI options
  boost::program_options::options_description opt_desc;
  opt_desc.add_options()
    ("help,h", "output help message and exit")
    ("config,c",
     boost::program_options::value<kwiver::vital::path_t>(),
     "Configuration file for the tool.")
    ("output-config,o",
     boost::program_options::value<kwiver::vital::path_t>(),
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

  namespace bfs = boost::filesystem;

  // Set up top level configuration w/ defaults where applicable.
  kwiver::vital::config_block_sptr config = default_config();
  kwiver::vital::algo::track_features_sptr feature_tracker;
  kwiver::vital::algo::image_io_sptr image_reader;
  kwiver::vital::algo::convert_image_sptr image_converter;
  kwiver::vital::algo::compute_ref_homography_sptr out_homog_generator;

  // If -c/--config given, read in confg file, merge in with default just generated
  if(vm.count("config"))
  {
    //std::cerr << "[DEBUG] Given config file: " << vm["config"].as<kwiver::vital::path_t>() << std::endl;
    config->merge_config(kwiver::vital::read_config_file(vm["config"].as<kwiver::vital::path_t>()));
  }

  //std::cerr << "[DEBUG] Config BEFORE set:" << std::endl;
  //print_config(config);

  kwiver::vital::algo::track_features::set_nested_algo_configuration("feature_tracker", config, feature_tracker);
  kwiver::vital::algo::track_features::get_nested_algo_configuration("feature_tracker", config, feature_tracker);
  kwiver::vital::algo::image_io::set_nested_algo_configuration("image_reader", config, image_reader);
  kwiver::vital::algo::image_io::get_nested_algo_configuration("image_reader", config, image_reader);
  kwiver::vital::algo::convert_image::set_nested_algo_configuration("convert_image", config, image_converter);
  kwiver::vital::algo::convert_image::get_nested_algo_configuration("convert_image", config, image_converter);
  kwiver::vital::algo::compute_ref_homography::set_nested_algo_configuration("output_homography_generator", config, out_homog_generator);
  kwiver::vital::algo::compute_ref_homography::get_nested_algo_configuration("output_homography_generator", config, out_homog_generator);

  //std::cerr << "[DEBUG] Config AFTER set:" << std::endl;
  //print_config(config);

  bool valid_config = check_config(config);

  if(vm.count("output-config"))
  {
    //std::cerr << "[DEBUG] Given config output target: " << vm["output-config"].as<kwiver::vital::path_t>() << std::endl;
    write_config_file(config, vm["output-config"].as<kwiver::vital::path_t>());
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
  //  - filepath validity checked above
  std::string image_list_file = config->get_value<std::string>("image_list_file");
  std::string mask_list_file = config->get_value<std::string>("mask_list_file");
  bool invert_masks = config->get_value<bool>("invert_masks");
  bool expect_multichannel_masks = config->get_value<bool>("expect_multichannel_masks");
  std::string output_tracks_file = config->get_value<std::string>("output_tracks_file");

  std::ifstream ifs(image_list_file.c_str());
  if (!ifs)
  {
    std::cerr << "Error: Could not open image list \""<<image_list_file<<"\""<<std::endl;
    return EXIT_FAILURE;
  }
  // Creating input image list, checking file existance
  std::vector<kwiver::vital::path_t> files;
  for (std::string line; std::getline(ifs,line); )
  {
    files.push_back(line);
    if (!bfs::exists(files[files.size()-1]))
    {
      throw kwiver::vital::path_not_exists(files[files.size()-1]);
    }
  }

  // Create mask image list if a list file was given, else fill list with empty
  // images. Files vector will only be populated if the use_masks bool is true
  bool use_masks = false;
  std::vector<kwiver::vital::path_t> mask_files;
  if( mask_list_file != "" )
  {
    LOG_DEBUG( main_logger,
               "Loading paired mask images from list file" );

    use_masks = true;
    // Load file stream
    std::ifstream mask_ifs(mask_list_file.c_str());
    if( !mask_ifs )
    {
      throw kwiver::vital::path_not_exists(mask_list_file);
    }
    // load filepaths from file
    for( std::string line; std::getline(mask_ifs, line); )
    {
      mask_files.push_back(line);
      if( !bfs::is_regular_file( mask_files[mask_files.size()-1] ) )
      {
        throw kwiver::vital::path_not_exists( mask_files[mask_files.size()-1] );
      }
    }
    // Check that image/mask list sizes are the same
    if( files.size() != mask_files.size() )
    {
      throw kwiver::vital::invalid_value("Image and mask file lists are not congruent "
                                 "in size.");
    }
    LOG_DEBUG( main_logger,
               "Loaded " << mask_files.size() << " mask image files." );
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
  if ( config->has_value("output_homography_file") &&
       config->get_value<std::string>("output_homography_file") != "" )
  {
    kwiver::vital::path_t homog_fp = config->get_value<kwiver::vital::path_t>("output_homography_file");
    homog_ofs.open( homog_fp.c_str() );
    if ( !homog_ofs )
    {
      std::cerr << "Error: Could not open homography file for writing: "
                << homog_fp
                << std::endl;
      return EXIT_FAILURE;
    }
  }

  // Track features on each frame sequentially
  kwiver::vital::track_set_sptr tracks;
  for(unsigned i=0; i<files.size(); ++i)
  {
    std::cout << "processing frame "<<i<<": "<<files[i]<<std::endl;

    auto const image = image_reader->load( files[i] );
    auto const converted_image = image_converter->convert( image );

    // Load the mask for this image if we were given a mask image list
    kwiver::vital::image_container_sptr mask, converted_mask;
    if( use_masks )
    {
      mask = image_reader->load( mask_files[i] );

      // error out if we are not expecting a multi-channel mask
      if( !expect_multichannel_masks && mask->depth() > 1 )
      {
        LOG_ERROR( main_logger,
                   "Encounted multi-channel mask image!" );
        return EXIT_FAILURE;
      }
      else if( expect_multichannel_masks && mask->depth() == 1 )
      {
        LOG_WARN( main_logger,
                  "Expecting multi-channel masks but received one that was "
                  "single-channel." );
      }

      if( invert_masks )
      {
        LOG_DEBUG( main_logger,
                   "Inverting mask image pixels" );
        kwiver::vital::image mask_image( mask->get_image() );
        kwiver::vital::transform_image( mask_image, invert_mask_pixel );
        LOG_DEBUG( main_logger,
                   "Inverting mask image pixels -- Done" );
      }

      converted_mask = image_converter->convert( mask );
    }

    tracks = feature_tracker->track(tracks, i, converted_image, converted_mask);
    if (tracks)
    {
      tracks = kwiver::maptk::extract_feature_colors(*tracks, *image, i);
    }

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
  kwiver::vital::write_track_file(tracks, output_tracks_file);

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
