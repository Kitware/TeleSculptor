/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include <iostream>
#include <fstream>
#include <exception>
#include <string>
#include <vector>

#include <maptk/modules.h>

#include <maptk/core/track_set_io.h>
#include <maptk/core/algo/image_io.h>
#include <maptk/core/algo/analyze_tracks.h>
#include <maptk/core/algo/draw_tracks.h>
#include <maptk/core/image_container.h>
#include <maptk/core/config_block.h>
#include <maptk/core/config_block_io.h>
#include <maptk/core/exceptions.h>
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
  maptk::config_block_sptr config =
    maptk::config_block::empty_config( "analyze_tracks_tool" );

  config->set_value( "track_file", "",
                     "Path to a required input file containing all features tracks "
                     "generated from some prior processing." );
  config->set_value( "image_list_file", "",
                     "Path to an optional input file containing new-line separated "
                     "paths to sequential image files for the given tracks. This "
                     "file is required for draw tracks output." );
  config->set_value( "output_file", "",
                     "Path to an optional file to write text outputs to. If this file "
                     "exists, it will be overwritten." );

  //maptk::algo::image_io::get_nested_algo_configuration(
  //  "image_reader", config, maptk::algo::image_io_sptr() );

  maptk::algo::analyze_tracks::get_nested_algo_configuration(
    "analyze_tracks", config, maptk::algo::analyze_tracks_sptr() );

  //maptk::algo::draw_tracks::get_nested_algo_configuration(
  //  "draw_tracks", config, maptk::algo::draw_tracks_sptr() );

  return config;
}


static bool check_config( maptk::config_block_sptr config )
{
  if( !config->has_value( "track_file" ) ||
      !maptk::algo::analyze_tracks::check_nested_algo_configuration( "analyze_tracks", config ) )
  {
    return false;
  }

  if( config->has_value( "image_list_file" ) &&
      ( !bfs::exists( maptk::path_t( config->get_value<std::string>( "image_list_file" ) ) ) ||
        !maptk::algo::image_io::check_nested_algo_configuration( "image_reader", config ) ||
        !maptk::algo::image_io::check_nested_algo_configuration( "draw_tracks", config ) ) )
  {
    return false;
  }

  return true;
}


static int maptk_main(int argc, char const* argv[])
{
  // register the algorithms in the various modules for dynamic look-up
  maptk::register_modules();

  // define/parse CLI options
  boost::program_options::options_description opt_desc;
  opt_desc.add_options()
    ( "help,h", "output help message and exit" )
    ( "config,c",
      boost::program_options::value<maptk::path_t>(),
      "Configuration file for the tool." )
    ( "output-config,o",
      boost::program_options::value<maptk::path_t>(),
      "Output a configuration.This may be seeded with a configuration file from -c/--config." )
    ;
  boost::program_options::variables_map vm;

  try
  {
    boost::program_options::store(
      boost::program_options::parse_command_line(argc, argv, opt_desc), vm );
  }
  catch( boost::program_options::unknown_option const& e )
  {
    std::cerr << "Error: unknown option " << e.get_option_name() << std::endl;
    return EXIT_FAILURE;
  }

  boost::program_options::notify( vm );

  if( vm.count( "help" ) )
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

  algo::image_io_sptr image_reader;
  algo::analyze_tracks_sptr analyze_tracks;
  algo::draw_tracks_sptr draw_tracks;

  // If -c/--config given, read in confgi file, merge in with default just generated
  if( vm.count( "config" ) )
  {
    config->merge_config( maptk::read_config_file( vm[ "config" ].as<maptk::path_t>() ) );
  }

  // Load all input images if they are specified
  bool use_images = config->has_value( "image_file_list" ) &&
                    !config->get_value<std::string>( "image_list_file" ).empty();

  bool output_to_file = config->has_value( "output_file" ) &&
                        !config->get_value<std::string>( "output_file" ).empty();

  if( use_images )
  {
    algo::image_io::set_nested_algo_configuration( "image_reader", config, image_reader );
    algo::image_io::get_nested_algo_configuration( "image_reader", config, image_reader );

    algo::draw_tracks::set_nested_algo_configuration( "draw_tracks", config, draw_tracks );
    algo::draw_tracks::get_nested_algo_configuration( "draw_tracks", config, draw_tracks );
  }

  algo::analyze_tracks::set_nested_algo_configuration( "analyze_tracks", config, analyze_tracks );
  algo::analyze_tracks::get_nested_algo_configuration( "analyze_tracks", config, analyze_tracks );

  bool valid_config = check_config( config );

  // Output a config file if specified
  if( vm.count( "output-config" ) )
  {
    write_config_file( config, vm[ "output-config" ].as<maptk::path_t>() );

    if( valid_config )
    {
      std::cerr << "INFO: Configuration file contained valid parameters and may be used "
                << "for running" << std::endl;
    }
    else
    {
      std::cerr << "WARNING: Configuration deemed not valid." << std::endl;
    }
    return EXIT_SUCCESS;
  }
  else if( !valid_config )
  {
    std::cerr << "ERROR: Configuration not valid." << std::endl;
    return EXIT_FAILURE;
  }

  // Load tracks
  std::string track_file = config->get_value<std::string>( "track_file" );
  maptk::track_set_sptr tracks;
  tracks = maptk::read_track_file( track_file );

  // Generate statistics if enabled
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

  // Read and process input images if set
  if( use_images )
  {
    maptk::image_container_sptr_list images;

    std::string image_list_file = config->get_value<std::string>( "image_list_file" );

    std::ifstream ifs( image_list_file.c_str() );

    if( !ifs )
    {
      std::cerr << "Error: Could not open image list \"" << image_list_file << "\"" << std::endl;
      return EXIT_FAILURE;
    }

    // Creating input image list, checking file existance, and loading the image
    for( std::string line; std::getline(ifs,line); )
    {
      if( !bfs::exists( line ) )
      {
        throw maptk::path_not_exists( line );
      }

      maptk::image_container_sptr image = image_reader->load( line );

      images.push_back( image );
    }

    draw_tracks->draw( tracks, images );
  }

  return EXIT_SUCCESS;
}


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
