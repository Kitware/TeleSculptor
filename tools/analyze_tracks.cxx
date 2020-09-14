/*ckwg +29
 * Copyright 2014-2017 by Kitware, Inc.
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

#include <iostream>
#include <fstream>
#include <exception>
#include <string>
#include <vector>

#include <vital/config/config_block.h>
#include <vital/config/config_block_io.h>

#include <vital/algo/analyze_tracks.h>
#include <vital/algo/draw_tracks.h>
#include <vital/algo/video_input.h>
#include <vital/exceptions.h>
#include <vital/io/camera_io.h>
#include <vital/io/camera_map_io.h>
#include <vital/io/landmark_map_io.h>
#include <vital/io/track_set_io.h>
#include <vital/plugin_loader/plugin_manager.h>
#include <vital/types/camera.h>
#include <vital/types/image_container.h>
#include <vital/types/landmark_map.h>
#include <vital/vital_types.h>
#include <vital/util/get_paths.h>

#include <kwiversys/SystemTools.hxx>
#include <kwiversys/CommandLineArguments.hxx>

#include <arrows/core/projected_track_set.h>
#include <maptk/version.h>

typedef kwiversys::SystemTools     ST;
typedef kwiversys::CommandLineArguments argT;


// Global options
bool        opt_help( false );
std::string opt_config;         // config file name
std::string opt_out_config;     // output config file name


// ------------------------------------------------------------------
static kwiver::vital::config_block_sptr default_config()
{
  kwiver::vital::config_block_sptr config =
    kwiver::vital::config_block::empty_config( "analyze_tracks_tool" );

  config->set_value( "track_file", "",
                     "Path to a required input file containing all features tracks "
                     "generated from some prior processing." );
  config->set_value( "video_source", "",
                     "Path to an input file to be opened as a video. "
                     "This could be either a video file or a text file "
                     "containing new-line separated paths to sequential "
                     "image files.");
  config->set_value( "output_file", "",
                     "Path to an optional file to write text outputs to. If this file "
                     "exists, it will be overwritten." );
  config->set_value( "comparison_track_file", "",
                     "Path to an optional second track input file containing tracks "
                     "which we want to compare against the first set." );
  config->set_value( "comparison_landmark_file", "",
                     "Path to an optional landmark ply file, which can be used along "
                     "with a camera file to generate a comparison track set." );
  config->set_value( "comparison_camera_dir", "",
                     "Path to an optional camera directory, which can be used alongside "
                     "a landmark ply file to generate a comparison track set." );

  kwiver::vital::algo::analyze_tracks::get_nested_algo_configuration(
    "track_analyzer", config, kwiver::vital::algo::analyze_tracks_sptr() );

  return config;
}


// ------------------------------------------------------------------
static bool check_config( kwiver::vital::config_block_sptr config )
{
  if( !config->has_value( "track_file" ) ||
      !ST::FileExists( kwiver::vital::path_t( config->get_value<std::string>( "track_file" ) ) ) )
  {
    std::cerr << "A valid track file must be specified!" << std::endl;
    return false;
  }

  if( !kwiver::vital::algo::analyze_tracks::check_nested_algo_configuration( "track_analyzer", config ) )
  {
    std::cerr << "Invalid analyze_tracks config" << std::endl;
    return false;
  }

  if( config->has_value( "video_source" ) &&
      !config->get_value<std::string>( "video_source" ).empty() )
  {
    if( !ST::FileExists( kwiver::vital::path_t( config->get_value<std::string>( "video_source" ) ) ) )
    {
      std::cerr << "Cannot find video_source file" << std::endl;
      return false;
    }
    else if( !kwiver::vital::algo::draw_tracks::check_nested_algo_configuration( "track_drawer", config ) )
    {
      std::cerr << "Unable to configure track drawer" << std::endl;
      return false;
    }
  }

  if( config->has_value( "comparison_landmark_file" ) !=
      config->has_value( "comparison_camera_dir" ) )
  {
    std::cerr << "Both a landmark and camera file must be specified to use either." << std::endl;
    return false;
  }

  return true;
}


// ------------------------------------------------------------------
static int maptk_main(int argc, char const* argv[])
{
  kwiversys::CommandLineArguments arg;

  arg.Initialize( argc, argv );

  arg.AddArgument( "--help",        argT::NO_ARGUMENT, &opt_help, "Display usage information" );
  arg.AddArgument( "-h",            argT::NO_ARGUMENT, &opt_help, "Display usage information" );
  arg.AddArgument( "--config",      argT::SPACE_ARGUMENT, &opt_config, "Configuration file for tool" );
  arg.AddArgument( "-c",            argT::SPACE_ARGUMENT, &opt_config, "Configuration file for tool" );
  arg.AddArgument( "--output-config", argT::SPACE_ARGUMENT, &opt_out_config,
                   "Output a configuration. This may be seeded with a configuration file from -c/--config." );
  arg.AddArgument( "-o",            argT::SPACE_ARGUMENT, &opt_out_config,
                   "Output a configuration. This may be seeded with a configuration file from -c/--config." );


  if ( ! arg.Parse() )
  {
    std::cerr << "Problem parsing arguments" << std::endl;
    return EXIT_FAILURE;
  }

  if ( opt_help )
  {
    std::cerr
      << "USAGE: " << argv[0] << " [OPTS]\n\n"
      << "Options:"
      << arg.GetHelp() << std::endl;
    return EXIT_SUCCESS;
  }

  // register the algorithm implementations
  auto& vpm = kwiver::vital::plugin_manager::instance();
  std::string rel_plugin_path = kwiver::vital::get_executable_path() + "/../lib/kwiver/plugins";
  vpm.add_search_path(rel_plugin_path);
  vpm.load_all_plugins(kwiver::vital::plugin_manager::plugin_type::ALGORITHMS);

  // Set config to algo chain
  // Get config from algo chain after set
  // Check config validity, store result
  //
  // If -o/--output-config given, output config result and notify of current (in)validity
  // Else error if provided config not valid.

  // Set up top level configuration w/ defaults where applicable.
  kwiver::vital::config_block_sptr config = default_config();

  kwiver::vital::algo::video_input_sptr video_reader;
  kwiver::vital::algo::analyze_tracks_sptr analyze_tracks;
  kwiver::vital::algo::draw_tracks_sptr draw_tracks;

  // If -c/--config given, read in confgi file, merge in with default just generated
  if( ! opt_config.empty() )
  {
    const std::string prefix = kwiver::vital::get_executable_path() + "/..";
    config->merge_config(kwiver::vital::read_config_file(opt_config, "telesculptor",
                                                         TELESCULPTOR_VERSION, prefix));
  }

  // Load all input images if they are specified
  bool use_images = config->has_value( "video_source" ) &&
                    !config->get_value<std::string>( "video_source" ).empty();

  bool output_to_file = config->has_value( "output_file" ) &&
                        !config->get_value<std::string>( "output_file" ).empty();

  if( use_images )
  {
    kwiver::vital::algo::video_input::set_nested_algo_configuration("video_reader", config, video_reader);
    kwiver::vital::algo::video_input::get_nested_algo_configuration("video_reader", config, video_reader);

    kwiver::vital::algo::draw_tracks::set_nested_algo_configuration( "track_drawer", config, draw_tracks );
    kwiver::vital::algo::draw_tracks::get_nested_algo_configuration( "track_drawer", config, draw_tracks );
  }

  kwiver::vital::algo::analyze_tracks::set_nested_algo_configuration( "track_analyzer", config, analyze_tracks );
  kwiver::vital::algo::analyze_tracks::get_nested_algo_configuration( "track_analyzer", config, analyze_tracks );

  bool valid_config = check_config( config );

  // Output a config file if specified
  if( ! opt_out_config.empty() )
  {
    write_config_file( config, opt_out_config );

    if( valid_config )
    {
      std::cerr << "INFO: Configuration file contained valid parameters and may be used "
                << "for running" << std::endl;
    }
    else
    {
      std::cerr << "WARNING: Configuration deemed not valid for running." << std::endl;
    }
    return EXIT_SUCCESS;
  }
  else if( !valid_config )
  {
    std::cerr << "ERROR: Configuration not valid." << std::endl;
    return EXIT_FAILURE;
  }

  // Load main track set
  kwiver::vital::track_set_sptr tracks;

  std::cout << std::endl << "Loading main track set file..." << std::endl;
  std::string track_file = config->get_value<std::string>( "track_file" );
  tracks = kwiver::vital::read_track_file( track_file );

  // Generate statistics if enabled
  if( analyze_tracks )
  {
    std::cout << std::endl << "Generating track statistics..." << std::endl;

    if( output_to_file )
    {
      std::string output_file = config->get_value<std::string>( "output_file" );
      std::ofstream ofs( output_file.c_str() );

      if( !ofs )
      {
        std::cerr << "Error: Could not open file " << output_file << " for writing." << std::endl;
        return EXIT_FAILURE;
      }

      analyze_tracks->print_info( tracks, ofs );

      ofs.close();
    }
    else
    {
      analyze_tracks->print_info( tracks, std::cout );
    }
  }

  // Read and process input images if set
  if( use_images )
  {
    std::vector<kwiver::vital::path_t> image_paths;
    std::string video_source = config->get_value<std::string>( "video_source" );

    video_reader->open(video_source);

    // Load comparison tracks if enabled
    kwiver::vital::track_set_sptr comparison_tracks;

    if( config->has_value( "comparison_track_file" ) &&
        !config->get_value<std::string>( "comparison_track_file" ).empty() )
    {
      track_file = config->get_value<std::string>( "comparison_track_file" );

      std::cout << std::endl << "Loading comparison track set file..." << std::endl;

      comparison_tracks = kwiver::vital::read_track_file( track_file );
    }
    else if( config->has_value( "comparison_landmark_file" ) &&
             !config->get_value<std::string>( "comparison_landmark_file" ).empty() &&
             config->has_value( "comparison_camera_dir" ) &&
             !config->get_value<std::string>( "comparison_camera_dir" ).empty() )
    {
      std::string landmark_file = config->get_value<std::string>( "comparison_landmark_file" );
      std::string camera_dir = config->get_value<std::string>( "comparison_camera_dir" );

      std::cout << std::endl << "Loading comparison track set file..." << std::endl;

      kwiver::vital::landmark_map_sptr landmarks = kwiver::vital::read_ply_file( landmark_file );
      kwiver::vital::camera_map_sptr cameras = kwiver::vital::read_krtd_files( image_paths, camera_dir );

      if( !cameras )
      {
        std::cerr << "Unable to load any camera files." << std::endl;
        return EXIT_FAILURE;
      }

      if( !landmarks )
      {
        std::cerr << "Unable to load landmark file." << std::endl;
        return EXIT_FAILURE;
      }

      comparison_tracks = kwiver::arrows::projected_tracks( landmarks, cameras );
    }

    // Read images one by one, this is more memory efficient than loading them all
    std::cout << std::endl << "Generating feature images..." << std::endl;

    kwiver::vital::timestamp ts;
    while( video_reader->next_frame(ts) )
    {
      kwiver::vital::image_container_sptr_list images;
      kwiver::vital::image_container_sptr image = video_reader->frame_image();
      images.push_back( image );

      // Draw tracks on images
      draw_tracks->draw( tracks, images, comparison_tracks );
    }
  }

  std::cout << std::endl;
  return EXIT_SUCCESS;
}


// ------------------------------------------------------------------
int main( int argc, char const* argv[] )
{
  try
  {
    return maptk_main( argc, argv );
  }
  catch( std::exception const& e )
  {
    std::cerr << "Exception caught: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }
  catch( ... )
  {
    std::cerr << "Unknown exception caught" << std::endl;
    return EXIT_FAILURE;
  }
}
