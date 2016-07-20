/*ckwg +29
 * Copyright 2016 by Kitware, Inc.
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
 * \brief Create undistorted frames from frames and KRTD files
 */

#include <iostream>
#include <fstream>
#include <exception>

#include <vital/config/config_block_io.h>

#include <vital/io/camera_io.h>
#include <vital/util/get_paths.h>

#include <kwiversys/SystemTools.hxx>
#include <kwiversys/CommandLineArguments.hxx>

#include <maptk/undistort_frame.h>
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
  config->set_value("image_list_file", "",
                    "Path to the input image list file.");

  config->set_value("input_krtd_dir", "",
                    "A directory containing the input KRTD files.");

  config->set_value("output_dir", "undistorted_frames",
                    "A directory in which to write the output undistorted frames.");

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

  kwiver::vital::path_t input_frames, input_krtd, output_dir;

  if(config->has_value("image_list_file"))
  {
    input_frames =
          config->get_value<kwiver::vital::path_t>("image_list_file");
  }

  if(config->has_value("input_krtd_dir"))
  {
    input_krtd =
          config->get_value<kwiver::vital::path_t>("input_krtd_dir");
  }

  if(config->has_value("output_dir"))
  {
    output_dir =
          config->get_value<kwiver::vital::path_t>("output_dir");
  }

  if (input_frames == "")
  {
    MAPTK_CHECK_FAIL("Not given an input frame list file.");
  }
  else if ( ! ST::FileExists(input_frames))
  {
    MAPTK_CHECK_FAIL("Path given for image_list_file doesn't exist.");
  }

  if (input_krtd == "")
  {
    MAPTK_CHECK_FAIL("Not given an input KRTD files directory.");
  }
  else if ( ! ST::FileExists(input_krtd))
  {
    MAPTK_CHECK_FAIL("Path given for input_krtd_dir doesn't exist.");
  }

  if (output_dir == "")
  {
    MAPTK_CHECK_FAIL("Not given an output directory.");
  }
  else if (! ST::FileExists(output_dir))
  {
    ST::MakeDirectory(output_dir);
  }
  // When we have a valid input path...
  else if (config_valid)
  {
    if ( ST::FileExists(input_frames) && ST::FileIsDirectory(input_frames))
    {
      MAPTK_CHECK_FAIL("image_list_file is a directory.");
    }
    else if (ST::FileExists(input_krtd) && !ST::FileIsDirectory(input_krtd))
    {
      MAPTK_CHECK_FAIL("input_krtd_dir is not a directory.");
    }
  }

#undef MAPTK_CHECK_FAIL

  return config_valid;
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
      << "Take a framelist and a krtd directory and return undistorted frames into specified output directory."
      << std::endl
      << "Options:"
      << arg.GetHelp() << std::endl;
    return EXIT_SUCCESS;
  }

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

  std::string image_list_file = config->get_value<std::string>("image_list_file");
  std::string input_krtd_dir = config->get_value<std::string>("input_krtd_dir");
  std::string output_dir= config->get_value<std::string>("output_dir");

  std::ifstream image_list_ifs(image_list_file.c_str());
  if (!image_list_ifs)
  {
    std::cerr << "Error: Could not open image list file!" << std::endl;
    return EXIT_FAILURE;
  }

  std::cerr << "Loading frames and cameras..." ;

  std::vector<kwiver::vital::path_t> image_files;
  std::vector<kwiver::vital::camera_sptr> cameras;
  kwiver::vital::path_t krtd_file;

  for (std::string line; std::getline(image_list_ifs, line); )
  {
    krtd_file = ST::ConvertToOutputPath(input_krtd_dir + "/" +
                                        ST::GetFilenameWithoutLastExtension(line)
                                        + ".krtd");

    auto const& camera = kwiver::vital::read_krtd_file(krtd_file);

    image_files.push_back(line);
    cameras.push_back(camera);
  }

  std::cerr << " Done." << std::endl;

  kwiver::maptk::undistortFrames(image_files,cameras,output_dir);

  std::cerr << " Done." << std::endl;

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
