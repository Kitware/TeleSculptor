/*ckwg +29
 * Copyright 2013-2017 by Kitware, Inc.
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
 * \brief POS file to KRTD conversion utility
 */

#include "tool_common.h"

#include <iostream>
#include <fstream>
#include <exception>
#include <string>
#include <vector>

#include <vital/config/config_block.h>
#include <vital/config/config_block_io.h>

#include <vital/algo/video_input.h>
#include <vital/io/camera_io.h>
#include <vital/io/camera_from_metadata.h>
#include <vital/io/eigen_io.h>
#include <vital/io/metadata_io.h>
#include <vital/exceptions.h>
#include <vital/plugin_loader/plugin_manager.h>
#include <vital/vital_types.h>
#include <vital/types/geodesy.h>
#include <vital/util/get_paths.h>

#include <kwiversys/SystemTools.hxx>
#include <kwiversys/CommandLineArguments.hxx>
#include <kwiversys/Directory.hxx>

#include <vital/types/local_geo_cs.h>
#include <maptk/version.h>

typedef kwiversys::SystemTools     ST;
typedef kwiversys::CommandLineArguments argT;

static kwiver::vital::logger_handle_t
  main_logger( kwiver::vital::get_logger( "pos2krtd_tool" ) );

// ------------------------------------------------------------------
// return a default configuration object
kwiver::vital::config_block_sptr
default_config()
{
  kwiver::vital::config_block_sptr config = kwiver::vital::config_block::empty_config();

  // general options
  config->set_value("video_source", "",
                    "Path to an input file to be opened as a video. "
                    "This could be either a video file or a text file "
                    "containing new-line separated paths to sequential "
                    "image files.  Only metadata is ready from the video.");

  config->set_value("output", "",
                    "Output file or directory where output files will be "
                    "placed. If a directory, output files will mirror the "
                    "filename stem of input files. The output file mode will "
                    "be interpreted the same as the file mode of the input "
                    "parameter.");

  config->set_value("geo_origin_file", "output/geo_origin.txt",
                    "This file contains the geographical location of the origin "
                    "of the local cartesian coordinate system used in the camera "
                    "and landmark files.  This file is use for input and output. "
                    "If the files exists it will be read to define the origin. "
                    "If the file does not exist an origin will be computed from "
                    "geographic metadata provided and written to this file. "
                    "The file format is ASCII (degrees, meters):\n"
                    "latitude longitude altitude");


  // base camera options
  config->set_value("base_camera:focal_length", "1.0",
                    "focal length of the base camera model");
  config->set_value("base_camera:principal_point", "640 480",
                    "The principal point of the base camera model \"x y\".\n"
                    "It is usually safe to assume this is the center of the "
                    "image.");
  config->set_value("base_camera:aspect_ratio", "1.0",
                    "the pixel aspect ratio of the base camera model");
  config->set_value("base_camera:skew", "0.0",
                    "The skew factor of the base camera model.\n"
                    "This is almost always zero in any real camera.");

  // INS transformation/offset options
  config->set_value("ins:rotation_offset", "0 0 0 1",
                    "A quaternion used to offset rotation data from POS files when "
                    "updating cameras. This is used to correct erroneous "
                    "metadata in the POS files.");

  auto default_vi = kwiver::vital::algo::video_input::create("pos");
  kwiver::vital::algo::video_input::
    get_nested_algo_configuration("video_reader", config, default_vi);

  return config;
}


// ------------------------------------------------------------------
/// Check configuration options
bool
check_config(kwiver::vital::config_block_sptr config)
{
  bool config_valid = true;

#define MAPTK_CHECK_FAIL(msg)                                   \
  std::cerr << "Config Check Fail: " << msg << std::endl;       \
  config_valid = false

  if (config->get_value<std::string>("video_source") == "")
  {
    MAPTK_CHECK_FAIL("Not given an video_source file or directory.");
  }
  else if ( ! ST::FileExists(config->get_value<kwiver::vital::path_t>("video_source")))
  {
    MAPTK_CHECK_FAIL("Path given for video_source doesn't exist.");
  }

  if (!config->has_value("output")
      || config->get_value<std::string>("output") == "")
  {
    MAPTK_CHECK_FAIL("Not given an output file or directory.");
  }
  // When we have a valid input path...
  else if (config_valid)
  {
    kwiver::vital::path_t video_source = config->get_value<kwiver::vital::path_t>("video_source"),
           output = config->get_value<kwiver::vital::path_t>("output");
    if ( ST::FileExists( output ) )
    {
      if (!ST::FileIsDirectory(output))
      {
        MAPTK_CHECK_FAIL("Output given exists but is not a directory!");
      }
    }
  }

#undef MAPTK_CHECK_FAIL

  return config_valid;
}


// ------------------------------------------------------------------
/// create a base camera instance from config options
kwiver::vital::simple_camera_perspective
base_camera_from_config(kwiver::vital::config_block_sptr config)
{
  kwiver::vital::simple_camera_intrinsics
      K(config->get_value<double>("focal_length"),
        config->get_value<kwiver::vital::vector_2d>("principal_point"),
        config->get_value<double>("aspect_ratio"),
        config->get_value<double>("skew"));
  return kwiver::vital::simple_camera_perspective(kwiver::vital::vector_3d(0,0,-1),
                                                  kwiver::vital::rotation_d(), K);
}


// ------------------------------------------------------------------
static int maptk_main(int argc, char const* argv[])
{
  static bool        opt_help(false);
  static std::string opt_config;
  static std::string opt_out_config;

  kwiversys::CommandLineArguments arg;

  arg.Initialize( argc, argv );
  typedef kwiversys::CommandLineArguments argT;

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
    std::cout
      << "USAGE: " << argv[0] << " [OPTS]\n\n"
      << "If multiple POS files are to be converted into KRTD " << std::endl
      << "files, it is recomended to use the directory arguments " << std::endl
      << "in order for the application to create a unified local " << std::endl
      << "coordinate system." << std::endl
      << std::endl
      << "Options:"
      << arg.GetHelp() << std::endl;
    return EXIT_SUCCESS;
  }


  // register the algorithm implementations
  auto& vpm = kwiver::vital::plugin_manager::instance();
  std::string rel_plugin_path = kwiver::vital::get_executable_path() + "/../lib/kwiver/plugins";
  vpm.add_search_path(rel_plugin_path);
  vpm.load_all_plugins(kwiver::vital::plugin_manager::plugin_type::ALGORITHMS);

  // Tell PROJ where to find its data files
  std::string rel_proj_path = kwiver::vital::get_executable_path() + "/../share/proj";
  if ( kwiversys::SystemTools::FileExists(rel_proj_path) &&
       kwiversys::SystemTools::FileIsDirectory(rel_proj_path) )
  {
    kwiversys::SystemTools::PutEnv("PROJ_LIB="+rel_proj_path);
  }

  //
  // Initialize from configuration
  //
  kwiver::vital::config_block_sptr config = default_config();
  kwiver::vital::algo::video_input_sptr video_reader;

  // If -c/--config given, read in confg file, merge in with default just generated
  if ( ! opt_config.empty())
  {
    const std::string prefix = kwiver::vital::get_executable_path() + "/..";
    config->merge_config(kwiver::vital::read_config_file(opt_config, "telesculptor",
                                                         TELESCULPTOR_VERSION, prefix));
  }

  kwiver::vital::algo::video_input::
    set_nested_algo_configuration("video_reader", config, video_reader);
  kwiver::vital::algo::video_input::
    get_nested_algo_configuration("video_reader", config, video_reader);

  bool config_is_valid = check_config(config);

  if ( ! opt_out_config.empty() )
  {
    kwiver::vital::path_t output_path = opt_out_config;
    kwiver::vital::write_config_file(config, output_path);

    if (config_is_valid)
    {
      std::cerr << "INFO: Configuration valid for running." << std::endl;
    }
    else
    {
      std::cerr << "WARNING: Configuration needs revision." << std::endl;
    }
    return EXIT_SUCCESS;
  }
  else if (!config_is_valid)
  {
    std::cerr << "ERROR: Configuration invalid." << std::endl;
    return EXIT_FAILURE;
  }


  kwiver::vital::path_t video_source = config->get_value<kwiver::vital::path_t>("video_source"),
                       output = config->get_value<kwiver::vital::path_t>("output");
  auto base_camera = base_camera_from_config(config->subblock_view("base_camera"));
  kwiver::vital::rotation_d ins_rot_offset = config->get_value<kwiver::vital::rotation_d>("ins:rotation_offset");


  if( kwiver::vital::get_geo_conv() == nullptr )
  {
    std::cerr << "No geographic conversion module available" << std::endl;
    return EXIT_FAILURE;
  }

  //
  // Create the local coordinate system
  //
  kwiver::vital::local_geo_cs local_cs;
  bool geo_origin_loaded_from_file = false;
  if (config->get_value<std::string>("geo_origin_file", "") != "")
  {
    kwiver::vital::path_t geo_origin_file = config->get_value<kwiver::vital::path_t>("geo_origin_file");
    // load the coordinates from a file if it exists
    if (ST::FileExists(geo_origin_file, true))
    {
      read_local_geo_cs_from_file(local_cs, geo_origin_file);
      LOG_INFO(main_logger, "Loaded origin point from: " << geo_origin_file);
      geo_origin_loaded_from_file = true;
    }
  }


  std::map<kwiver::vital::frame_id_t, kwiver::vital::metadata_sptr> md_map;
  std::map<kwiver::vital::frame_id_t, std::string> krtd_filenames;

  LOG_INFO( main_logger, "Opening Video: " << video_source );
  video_reader->open(video_source);

  LOG_INFO( main_logger, "Reading Video" );
  kwiver::vital::timestamp ts;
  while( video_reader->next_frame(ts) )
  {
    auto md_vec = video_reader->frame_metadata();
    if( md_vec.empty() || !md_vec[0] )
    {
      continue;
    }
    auto md = md_vec[0];
    md_map[ts.get_frame()] = md;
    std::string basename = kwiver::vital::basename_from_metadata(md, ts.get_frame());
    krtd_filenames[ts.get_frame()] = output + "/" + basename + ".krtd";
  }

  if (md_map.size() == 0)
  {
    LOG_WARN( main_logger, "No valid metadata found in directory. Nothing to do.");
    return EXIT_SUCCESS;
  }

  LOG_INFO( main_logger, "Initializing cameras" );
  std::map<kwiver::vital::frame_id_t, kwiver::vital::camera_sptr> cam_map;
  cam_map = kwiver::vital::initialize_cameras_with_metadata(
    md_map, base_camera, local_cs, true, ins_rot_offset);

  // create output KRTD directory
  if( ! ST::FileExists(output) )
  {
    if( ! ST::MakeDirectory( output ) )
    {
      LOG_ERROR( main_logger, "Unable to create output directory: " << output );
      return EXIT_FAILURE;
    }
  }

  LOG_INFO( main_logger, "Writing KRTD files" );
  typedef std::map<kwiver::vital::frame_id_t, kwiver::vital::camera_sptr>::value_type cam_map_val_t;
  for(cam_map_val_t const &p : cam_map)
  {
    auto cam = dynamic_cast<kwiver::vital::simple_camera_perspective*>(p.second.get());
    kwiver::vital::write_krtd_file(*cam, krtd_filenames[p.first]);
  }


  // if we computed an origin that was not loaded from a file
  if (!local_cs.origin().is_empty() &&
      !geo_origin_loaded_from_file &&
      config->get_value<std::string>("geo_origin_file", "") != "")
  {
    kwiver::vital::path_t geo_origin_file = config->get_value<kwiver::vital::path_t>("geo_origin_file");
    LOG_INFO(main_logger, "Saving local coordinate origin to " << geo_origin_file);
    write_local_geo_cs_to_file(local_cs, geo_origin_file);
  }

  return EXIT_SUCCESS;
}


// ------------------------------------------------------------------
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
