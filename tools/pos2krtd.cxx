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
 * \brief POS file to KRTD conversion utility
 */

#include <iostream>
#include <fstream>
#include <exception>
#include <string>
#include <vector>

#include <vital/vital_foreach.h>
#include <vital/config/config_block.h>
#include <vital/config/config_block_io.h>

#include <vital/io/camera_io.h>
#include <vital/io/eigen_io.h>
#include <vital/exceptions.h>
#include <vital/algorithm_plugin_manager.h>
#include <vital/vital_types.h>
#include <vital/util/get_paths.h>

#include <kwiversys/SystemTools.hxx>
#include <kwiversys/CommandLineArguments.hxx>
#include <kwiversys/Directory.hxx>

#include <maptk/ins_data_io.h>
#include <maptk/local_geo_cs.h>
#include <maptk/version.h>

typedef kwiversys::SystemTools     ST;
typedef kwiversys::CommandLineArguments argT;

// ------------------------------------------------------------------
// return a default configuration object
kwiver::vital::config_block_sptr
default_config()
{
  kwiver::vital::config_block_sptr config = kwiver::vital::config_block::empty_config();

  // general options
  config->set_value("input", "",
                    "Input file or directory of input files.\n"
                    "\n"
                    "If multiple POS files are to be converted into KRTD "
                    "files, it is recommended to use the directory arguemnts "
                    "in order for the application to create a unified local "
                    "coordinate system.");
  config->set_value("output", "",
                    "Output file or directory where output files will be "
                    "placed. If a directory, output files will mirror the "
                    "filename stem of input files. The output file mode will "
                    "be interpreted the same as the file mode of the input "
                    "parameter.\n"
                    "\n"
                    "I.e. if a file was provided for input, output "
                    "should point to a file path to output to. If input was a "
                    "directory, output will be treated like a directory.");

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
                    "updating cameras.");

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

  if (!config->has_value("input")
      || config->get_value<std::string>("input") == "")
  {
    MAPTK_CHECK_FAIL("Not given an input file or directory.");
  }
  else if ( ! ST::FileExists(config->get_value<kwiver::vital::path_t>("input")))
  {
    MAPTK_CHECK_FAIL("Path given for input doesn't exist.");
  }

  if (!config->has_value("output")
      || config->get_value<std::string>("output") == "")
  {
    MAPTK_CHECK_FAIL("Not given an output file or directory.");
  }
  // When we have a valid input path...
  else if (config_valid)
  {
    kwiver::vital::path_t input = config->get_value<kwiver::vital::path_t>("input"),
           output = config->get_value<kwiver::vital::path_t>("output");
    if ( ST::FileExists( output ) )
    {
      if (ST::FileIsDirectory(input) && !ST::FileIsDirectory(output))
      {
        MAPTK_CHECK_FAIL("Output given exists but is not a directory! "
                         "Input was a directory, so output must also be a "
                         "directory.");
      }
      else if (!ST::FileIsDirectory(input) && ST::FileIsDirectory(output))
      {
        MAPTK_CHECK_FAIL("Input was a file, but given output path is a "
                         "directory!");
      }
    }
  }

#undef MAPTK_CHECK_FAIL

  return config_valid;
}


// ------------------------------------------------------------------
/// create a base camera instance from config options
kwiver::vital::simple_camera
base_camera_from_config(kwiver::vital::config_block_sptr config)
{
  kwiver::vital::simple_camera_intrinsics
      K(config->get_value<double>("focal_length"),
        config->get_value<kwiver::vital::vector_2d>("principal_point"),
        config->get_value<double>("aspect_ratio"),
        config->get_value<double>("skew"));
  return kwiver::vital::simple_camera(kwiver::vital::vector_3d(0,0,-1),
                                      kwiver::vital::rotation_d(), K);
}


// ------------------------------------------------------------------
/// Convert a INS data to a camera
bool convert_ins2camera(const kwiver::maptk::ins_data& ins,
                        kwiver::maptk::local_geo_cs& cs,
                        kwiver::vital::simple_camera& cam,
                        kwiver::vital::rotation_d const& ins_rot_offset = kwiver::vital::rotation_d())
{
  if( cs.utm_origin_zone() < 0 )
  {
    std::cerr << "lat: "<<ins.lat<<" lon: "<<ins.lon<<std::endl;
    cs.set_utm_origin_zone(cs.geo_map_algo()->latlon_zone(ins.lat, ins.lon));
    std::cerr << "using zone "<< cs.utm_origin_zone() <<std::endl;
  }

  cs.update_camera(ins, cam, ins_rot_offset);
  return true;
}


// ------------------------------------------------------------------
/// Convert a POS file to a KRTD file
bool convert_pos2krtd(const kwiver::vital::path_t& pos_filename,
                      const kwiver::vital::path_t& krtd_filename,
                      kwiver::maptk::local_geo_cs& cs,
                      const kwiver::vital::simple_camera& base_camera,
                      kwiver::vital::rotation_d const& ins_rot_offset = kwiver::vital::rotation_d())
{
  kwiver::maptk::ins_data ins;
  // a local copy of the base camera to modify in place
  kwiver::vital::simple_camera local_camera(base_camera);
  ins = kwiver::maptk::read_pos_file(pos_filename);
  if ( !convert_ins2camera(ins, cs, local_camera, ins_rot_offset) )
  {
    return false;
  }
  kwiver::vital::write_krtd_file(local_camera, krtd_filename);
  return true;
}


// ------------------------------------------------------------------
/// Convert a directory of POS file to a directory of KRTD files
bool convert_pos2krtd_dir(const kwiver::vital::path_t& pos_dir,
                          const kwiver::vital::path_t& krtd_dir,
                          kwiver::maptk::local_geo_cs& cs,
                          const kwiver::vital::simple_camera& base_camera,
                          kwiver::vital::rotation_d const& ins_rot_offset = kwiver::vital::rotation_d())
{
  std::vector<std::string> krtd_filenames;
  std::map<kwiver::vital::frame_id_t, kwiver::maptk::ins_data> ins_map;

  kwiversys::Directory dir;
  dir.Load( pos_dir );

  std::cerr << "Loading POS files" << std::endl;

  unsigned long num_files = dir.GetNumberOfFiles();
  for ( unsigned long i = 0; i < num_files; i++)
  {
    kwiver::vital::path_t p = pos_dir + "/" + dir.GetFile( i );

    try
    {
      kwiver::maptk::ins_data ins = kwiver::maptk::read_pos_file( p );

      kwiver::vital::path_t krtd_filename = krtd_dir + "/"
                                          + ST::GetFilenameWithoutLastExtension( p ) + ".krtd";
      //std::cerr << "Loading " << p << std::endl;
      kwiver::vital::frame_id_t frame = static_cast<kwiver::vital::frame_id_t>(krtd_filenames.size());
      ins_map[frame] = ins;
      krtd_filenames.push_back(krtd_filename);
    }
    catch (kwiver::vital::io_exception const& e)
    {
      std::cerr << "-> Skipping invalid file: " << p << std::endl;
      std::cerr << "   " << e.what() << std::endl;
    }
  } // end foreach

  if (ins_map.size() == 0)
  {
    std::cerr << "WARNING: No valid input files found in directory. "
              << "Nothing to do."
              << std::endl;
    return false;
  }

  std::cerr << "Initializing cameras" << std::endl;
  std::map<kwiver::vital::frame_id_t, kwiver::vital::camera_sptr> cam_map;
  cam_map = kwiver::maptk::initialize_cameras_with_ins(ins_map, base_camera, cs, ins_rot_offset);

  std::cerr << "Writing KRTD files" << std::endl;
  typedef std::map<kwiver::vital::frame_id_t, kwiver::vital::camera_sptr>::value_type cam_map_val_t;
  VITAL_FOREACH(cam_map_val_t const &p, cam_map)
  {
    kwiver::vital::simple_camera* cam = dynamic_cast<kwiver::vital::simple_camera*>(p.second.get());
    kwiver::vital::write_krtd_file(*cam, krtd_filenames[p.first]);
  }

  kwiver::vital::vector_3d origin = cs.utm_origin();
  std::cerr << "using local UTM origin at "<<origin[0] <<", "<<origin[1]
            <<", zone "<<cs.utm_origin_zone() <<std::endl;
  return true;
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
  std::string rel_plugin_path = kwiver::vital::get_executable_path() + "/../lib/maptk";
  kwiver::vital::algorithm_plugin_manager::instance().add_search_path(rel_plugin_path);
  kwiver::vital::algorithm_plugin_manager::instance().register_plugins();

  //
  // Initialize from configuration
  //
  kwiver::vital::config_block_sptr config = default_config();

  if ( ! opt_config.empty())
  {
    const std::string prefix = kwiver::vital::get_executable_path() + "/..";
    config->merge_config(kwiver::vital::read_config_file(opt_config, "maptk",
                                                         MAPTK_VERSION, prefix));
  }

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


  kwiver::vital::path_t input = config->get_value<kwiver::vital::path_t>("input"),
                output = config->get_value<kwiver::vital::path_t>("output");
  kwiver::vital::simple_camera base_camera = base_camera_from_config(config->subblock_view("base_camera"));
  kwiver::vital::rotation_d ins_rot_offset = config->get_value<kwiver::vital::rotation_d>("ins:rotation_offset");


  kwiver::vital::algo::geo_map_sptr geo_mapper = kwiver::vital::algo::geo_map::create("proj");
  if( !geo_mapper )
  {
    std::cerr << "No geo_map module available" << std::endl;
    return EXIT_FAILURE;
  }

  kwiver::maptk::local_geo_cs local_cs(geo_mapper);

  if( ST::FileIsDirectory(input) )
  {
    std::cerr << "processing "<<input<<" as a directory of POS files" << std::endl;
    if( ! ST::FileExists(output) )
    {
      if( ! ST::MakeDirectory( output ) )
      {
        std::cerr << "Unable to create output directory: " << output << std::endl;
        return EXIT_FAILURE;
      }
    }
    if( !convert_pos2krtd_dir(input, output, local_cs, base_camera, ins_rot_offset) )
    {
      return EXIT_FAILURE;
    }
  }
  // otherwise treat input as a single file
  else if( !convert_pos2krtd(input, output, local_cs, base_camera, ins_rot_offset) )
  {
    return EXIT_FAILURE;
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
