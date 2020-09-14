/*ckwg +29
 * Copyright 2013-2020 by Kitware, Inc.
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
 * \brief Feature detector and descriptor utility
 */


#include "tool_common.h"

#include <deque>
#include <iostream>
#include <fstream>
#include <exception>
#include <string>
#include <vector>

#include <arrows/core/colorize.h>

#include <vital/config/config_block.h>
#include <vital/config/config_block_io.h>
#include <vital/logger/logger.h>

#include <vital/exceptions.h>
#include <vital/io/metadata_io.h>
#include <vital/io/track_set_io.h>
#include <vital/vital_types.h>
#include <vital/algo/image_io.h>
#include <vital/algo/convert_image.h>
#include <vital/algo/detect_features.h>
#include <vital/algo/extract_descriptors.h>
#include <vital/algo/feature_descriptor_io.h>
#include <vital/algo/video_input.h>
#include <vital/plugin_loader/plugin_manager.h>
#include <vital/util/get_paths.h>
#include <vital/util/thread_pool.h>
#include <vital/util/transform_image.h>

#include <kwiversys/SystemTools.hxx>
#include <kwiversys/CommandLineArguments.hxx>

#include <maptk/version.h>

typedef kwiversys::SystemTools ST;
typedef kwiversys::CommandLineArguments argT;

static kwiver::vital::logger_handle_t main_logger( kwiver::vital::get_logger( "detect_and_describe_tool" ) );

// ------------------------------------------------------------------
static kwiver::vital::config_block_sptr default_config()
{
  kwiver::vital::config_block_sptr config = kwiver::vital::config_block::empty_config("detect_and_describe_tool");

  config->set_value("video_source", "",
                    "Path to an input file to be opened as a video. "
                    "This could be either a video file or a text file "
                    "containing new-line separated paths to sequential "
                    "image files.");
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
  config->set_value("features_dir", "",
                    "Path to a directory in which to write the output feature "
                    "detection and description files");
  config->set_value("validate_existing_features", true,
                    "When a features file already exists, validate that the "
                    "file can load sucessfully before deciding to skip "
                    "computation on this frame.  If this option is disabled "
                    "then skip if the file exists, without loading it");

  kwiver::vital::algo::video_input::get_nested_algo_configuration("video_reader", config,
                                      kwiver::vital::algo::video_input_sptr());
  kwiver::vital::algo::detect_features::get_nested_algo_configuration("feature_detector", config,
                                      kwiver::vital::algo::detect_features_sptr());
  kwiver::vital::algo::extract_descriptors::get_nested_algo_configuration("descriptor_extractor", config,
                                      kwiver::vital::algo::extract_descriptors_sptr());
  kwiver::vital::algo::image_io::get_nested_algo_configuration("image_reader", config,
                                      kwiver::vital::algo::image_io_sptr());
  kwiver::vital::algo::convert_image::get_nested_algo_configuration("convert_image", config,
                                      kwiver::vital::algo::convert_image_sptr());
  kwiver::vital::algo::feature_descriptor_io::get_nested_algo_configuration("fd_io", config,
                                      kwiver::vital::algo::feature_descriptor_io_sptr());
  return config;
}


// ------------------------------------------------------------------
static bool check_config(kwiver::vital::config_block_sptr config)
{
  bool config_valid = true;

#define MAPTK_CONFIG_FAIL(msg) \
  LOG_ERROR(main_logger, "Config Check Fail: " << msg); \
  config_valid = false

  // A given output directory is invalid if it names a file
  if ( config->has_value("features_dir")
    && config->get_value<std::string>("features_dir") != "" )
  {
    kwiver::vital::config_path_t fp = config->get_value<kwiver::vital::config_path_t>("features_dir");
    if ( ST::FileExists( fp ) && !ST::FileIsDirectory( fp ) )
    {
      MAPTK_CONFIG_FAIL("Given features directory is a file "
                        << "(Given: " << fp << ")");
    }

    // Check that fd_io algo is correctly configured
    if( !kwiver::vital::algo::feature_descriptor_io
             ::check_nested_algo_configuration("fd_io", config) )
    {
      MAPTK_CONFIG_FAIL("fd_io configuration check failed");
    }
  }

  if ( ! config->has_value("video_source") ||
      config->get_value<std::string>("video_source") == "")
  {
    MAPTK_CONFIG_FAIL("Config needs value video_source");
  }
  else
  {
    std::string path = config->get_value<std::string>("video_source");
    if ( ! ST::FileExists( kwiver::vital::path_t(path), true ) )
    {
      MAPTK_CONFIG_FAIL("video_source path, " << path << ", does not exist or is not a regular file");
    }
  }

  // If given an mask image list file, check that the file exists and is a file
  if (config->has_value("mask_list_file") && config->get_value<std::string>("mask_list_file") != "" )
  {
    std::string mask_list_file = config->get_value<std::string>("mask_list_file");
    if (mask_list_file != "" && ! ST::FileExists( kwiver::vital::path_t(mask_list_file), true ))
    {
      MAPTK_CONFIG_FAIL("mask_list_file path, " << mask_list_file << ", does not exist");
    }
  }

  if (!kwiver::vital::algo::video_input::check_nested_algo_configuration("video_reader", config))
  {
    MAPTK_CONFIG_FAIL("video_reader configuration check failed");
  }

  if (!kwiver::vital::algo::detect_features::check_nested_algo_configuration("feature_detector", config))
  {
    MAPTK_CONFIG_FAIL("feature_tracker configuration check failed");
  }

  if (!kwiver::vital::algo::extract_descriptors::check_nested_algo_configuration("descriptor_extractor", config))
  {
    MAPTK_CONFIG_FAIL("descriptor_extractor configuration check failed");
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


/// Check a filepath to see if it is an existing and valid KWFD file
/**
 * If a feature_descriptor_io algorithm is not specified only check that the
 * file exists.  Otherwise read the file and make sure it is valid.
 */
bool valid_feature_file_exists( std::string const& filepath,
                                kwiver::vital::frame_id_t frame,
                                kwiver::vital::algo::feature_descriptor_io_sptr fd_io )
{
  // if the features file already exists then test loading it and skip
  if( ST::FileExists( filepath ) )
  {
    if( !fd_io )
    {
      LOG_INFO( main_logger, "Skipping frame " << frame <<
                             ", output exists: " << filepath );
      return true;
    }
    try
    {
      kwiver::vital::feature_set_sptr feat;
      kwiver::vital::descriptor_set_sptr desc;
      fd_io->load(filepath, feat, desc);
      if( feat && feat->size() > 0 && desc && desc->size() > 0 )
      {
        LOG_INFO( main_logger, "Skipping frame " << frame <<
                               ", output exists: " << filepath <<
                               "\nfile contains " << feat->size() << " features, "
                               << desc->size() << " descriptors" );
        return true;
      }
    }
    catch(...)
    {
      LOG_WARN( main_logger, "Not able to load " << filepath << ", recomputing" );
    }
  }
  return false;
}


/// Extract the mask image from the container, invert, and repackage
kwiver::vital::image_container_sptr
invert_mask_image(kwiver::vital::image_container_sptr mask)
{
  LOG_DEBUG( main_logger,
             "Inverting mask image pixels" );
  kwiver::vital::image_of<bool> mask_image;
  kwiver::vital::cast_image( mask->get_image(), mask_image );
  kwiver::vital::transform_image( mask_image, [] (bool b) { return !b; } );
  LOG_DEBUG( main_logger,
             "Inverting mask image pixels -- Done" );
  return std::make_shared<kwiver::vital::simple_image_container>( mask_image );
}


/// Validate a mask image according to the expected number of channels.
bool validate_mask_image( kwiver::vital::image_container_sptr mask,
                          bool expect_multichannel_masks=false )
{
  // error out if we are not expecting a multi-channel mask
  if( !expect_multichannel_masks && mask->depth() > 1 )
  {
    LOG_ERROR( main_logger,
               "Encounted multi-channel mask image!" );
    return false;
  }
  else if( expect_multichannel_masks && mask->depth() == 1 )
  {
    LOG_WARN( main_logger,
              "Expecting multi-channel masks but received one that was "
              "single-channel." );
  }
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
  arg.AddArgument( "-h",            argT::NO_ARGUMENT, &opt_help, "Display usage information" );
  arg.AddArgument( "--config",      argT::SPACE_ARGUMENT, &opt_config, "Configuration file for tool" );
  arg.AddArgument( "-c",            argT::SPACE_ARGUMENT, &opt_config, "Configuration file for tool" );
  arg.AddArgument( "--output-config", argT::SPACE_ARGUMENT, &opt_out_config,
                   "Output a configuration. This may be seeded with a configuration file from -c/--config." );
  arg.AddArgument( "-o",            argT::SPACE_ARGUMENT, &opt_out_config,
                   "Output a configuration. This may be seeded with a configuration file from -c/--config." );

    if ( ! arg.Parse() )
  {
    LOG_ERROR(main_logger, "Problem parsing arguments");
    return EXIT_FAILURE;
  }

  if ( opt_help )
  {
    std::cout
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
  kwiver::vital::algo::detect_features_sptr feature_detector;
  kwiver::vital::algo::extract_descriptors_sptr descriptor_extractor;
  kwiver::vital::algo::feature_descriptor_io_sptr fd_io;
  kwiver::vital::algo::image_io_sptr image_reader;
  kwiver::vital::algo::convert_image_sptr image_converter;

  // If -c/--config given, read in confg file, merge in with default just generated
  if( ! opt_config.empty() )
  {
    const std::string prefix = kwiver::vital::get_executable_path() + "/..";
    config->merge_config(kwiver::vital::read_config_file(opt_config, "telesculptor",
                                                         TELESCULPTOR_VERSION, prefix));
  }

  kwiver::vital::algo::video_input::
    set_nested_algo_configuration("video_reader", config, video_reader);
  kwiver::vital::algo::video_input::
    get_nested_algo_configuration("video_reader", config, video_reader);
  kwiver::vital::algo::detect_features::
    set_nested_algo_configuration("feature_detector", config, feature_detector);
  kwiver::vital::algo::detect_features::
    get_nested_algo_configuration("feature_detector", config, feature_detector);
  kwiver::vital::algo::extract_descriptors::
    set_nested_algo_configuration("descriptor_extractor", config, descriptor_extractor);
  kwiver::vital::algo::extract_descriptors::
    get_nested_algo_configuration("descriptor_extractor", config, descriptor_extractor);
  kwiver::vital::algo::feature_descriptor_io::
    set_nested_algo_configuration("fd_io", config, fd_io);
  kwiver::vital::algo::feature_descriptor_io::
    get_nested_algo_configuration("fd_io", config, fd_io);
  kwiver::vital::algo::image_io::
    set_nested_algo_configuration("image_reader", config, image_reader);
  kwiver::vital::algo::image_io::
    get_nested_algo_configuration("image_reader", config, image_reader);
  kwiver::vital::algo::convert_image::
    set_nested_algo_configuration("convert_image", config, image_converter);
  kwiver::vital::algo::convert_image::
    get_nested_algo_configuration("convert_image", config, image_converter);

  bool valid_config = check_config(config);

  if( ! opt_out_config.empty() )
  {
    write_config_file(config, opt_out_config );
    if(valid_config)
    {
      LOG_INFO(main_logger, "Configuration file contained valid parameters and may be used for running");
    }
    else
    {
      LOG_WARN(main_logger, "Configuration deemed not valid.");
    }
    return EXIT_SUCCESS;
  }
  else if(!valid_config)
  {
    LOG_ERROR(main_logger, "Configuration not valid.");
    return EXIT_FAILURE;
  }

  // Attempt opening input and output files.
  //  - filepath validity checked above
  std::string video_source = config->get_value<std::string>("video_source");
  std::string mask_list_file = config->get_value<std::string>("mask_list_file");
  bool invert_masks = config->get_value<bool>("invert_masks");
  bool expect_multichannel_masks = config->get_value<bool>("expect_multichannel_masks");
  std::string features_dir = config->get_value<std::string>("features_dir");
  bool validate_existing_features = config->get_value<bool>("validate_existing_features");


  LOG_INFO( main_logger, "Reading Video" );
  video_reader->open(video_source);

  // Pre-scan the video to get an accurate frame count
  // We may wish to remove this later if we start operating on live streams
  kwiver::vital::timestamp ts;
  std::vector<kwiver::vital::timestamp> timestamps;
  while( video_reader->next_frame(ts) )
  {
    timestamps.push_back(ts);
  }
  // close and re-open to return to the video start
  video_reader->close();
  video_reader->open(video_source);


  // Create mask image list if a list file was given, else fill list with empty
  // images. Files vector will only be populated if the use_masks bool is true
  bool use_masks = false;
  std::vector<kwiver::vital::path_t> mask_files;
  if( mask_list_file != "" )
  {
    LOG_DEBUG( main_logger, "Checking paired mask images from list file" );

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
      if( ! ST::FileExists( mask_files[mask_files.size()-1], true ) )
      {
        throw kwiver::vital::path_not_exists( mask_files[mask_files.size()-1] );
      }
    }
    // Check that image/mask list sizes are the same
    if( timestamps.size() != mask_files.size() )
    {
      throw kwiver::vital::invalid_value("video and mask file lists have "
                                         "different frame counts");
    }
    LOG_DEBUG( main_logger,
               "Validated " << mask_files.size() << " mask image files." );
  }

  // Verify that the output directory exists, or make it
  // If the given path is a directory, we obviously can't write to it.
  if( ST::FileExists( features_dir )  && !ST::FileIsDirectory( features_dir ) )
  {
    throw kwiver::vital::file_write_exception(features_dir, "The output directory is a file");
  }

  // Check that the directory of the given filepath exists, creating necessary
  // directories where needed.
  if( ! ST::FileIsDirectory( features_dir ) )
  {
    if( ! ST::MakeDirectory( features_dir ) )
    {
      throw kwiver::vital::file_write_exception(features_dir, "Attempted directory creation, "
                                                "but no directory created!");
    }
  }

  std::mutex video_mutex;

  // This lambda function runs in a thread to read the video and launch processing jobs
  auto handle_frame = [&] ()
  {
    kwiver::vital::metadata_vector md_vec;
    kwiver::vital::image_container_sptr image;
    kwiver::vital::timestamp ts;
    {
      // lock the video mutex while incrementing the video and getting a frame
      std::lock_guard<std::mutex> vlock(video_mutex);
      if( !video_reader->next_frame(ts) )
      {
        return false;
      }
      md_vec = video_reader->frame_metadata();
      // get the frame now even though we do not yet know if it will be needed.
      // This way we can release the lock and let another thread increment the video.
      image = video_reader->frame_image();
    }

    kwiver::vital::metadata_sptr md;
    if( !md_vec.empty() )
    {
      md = md_vec[0];
    }
    std::string basename = kwiver::vital::basename_from_metadata(md, ts.get_frame());
    kwiver::vital::path_t kwfd_file = features_dir + "/" + basename + ".kwfd";

    // if the features file already exists then test loading it and skip
    if( valid_feature_file_exists( kwfd_file, ts.get_frame(),
                                   validate_existing_features ? fd_io : nullptr ) )
    {
      return true;
    }

    LOG_DEBUG(main_logger, "processing frame "<< ts.get_frame());

    auto const converted_image = image_converter->convert( image );

    // Load the mask for this image if we were given a mask image list
    kwiver::vital::image_container_sptr mask, converted_mask;
    if( use_masks )
    {
      mask = image_reader->load( mask_files[ts.get_frame()] );

      if( !validate_mask_image( mask, expect_multichannel_masks ) )
      {
        return false;
      }

      if( invert_masks )
      {
        mask = invert_mask_image( mask );
      }

      converted_mask = image_converter->convert( mask );
    }

    // detect features on the current frame
    kwiver::vital::feature_set_sptr curr_feat =
      feature_detector->detect(converted_image, converted_mask);

    LOG_INFO( main_logger, "Detected " << curr_feat->size() <<
                           " features on frame " << ts.get_frame() );

    if (curr_feat)
    {
      curr_feat = kwiver::arrows::core::extract_feature_colors(*curr_feat, *converted_image);
    }

    // extract descriptors on the current frame
    kwiver::vital::descriptor_set_sptr curr_desc =
      descriptor_extractor->extract(converted_image, curr_feat, converted_mask);

    LOG_DEBUG( main_logger, "Saving features to " << kwfd_file );
    // make the enclosing directory if it does not already exist
    const kwiver::vital::path_t fd_dir = ST::GetFilenamePath( kwfd_file );
    if( !ST::FileIsDirectory( fd_dir ) )
    {
      if( !ST::MakeDirectory( fd_dir ) )
      {
        LOG_ERROR( main_logger, "Unable to create directory: " << fd_dir );
        return false;
      }
    }
    fd_io->save(kwfd_file, curr_feat, curr_desc);

    return true;
  };



  // access the thread pool
  auto& pool = kwiver::vital::thread_pool::instance();

  // queue of future returns from jobs
  std::deque<std::future<bool> > frame_status_queue;

  // number of jobs to keep in the queue
  // this has 1.5 times the number of threads as a heuristic to make sure
  // we keep threads busy but don't lag too far in checking for errors
  size_t buffer = pool.num_threads() * 3 / 2;
  bool not_failed = true;
  while(not_failed)
  {
    // enqueue a task to process one frame
    frame_status_queue.push_back(pool.enqueue(handle_frame));
    // if we have the specified number of frames queued up
    if( frame_status_queue.size() > buffer )
    {
      // wait for a job to complete and get the status
      not_failed = frame_status_queue.front().get();
      frame_status_queue.pop_front();
    }
  }

  // wait for all remaining jobs to complete
  while( !frame_status_queue.empty() )
  {
    frame_status_queue.front().wait();
    frame_status_queue.pop_front();
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
    LOG_ERROR(main_logger, "Exception caught: " << e.what());

    return EXIT_FAILURE;
  }
  catch (...)
  {
    LOG_ERROR(main_logger, "Unknown exception caught");

    return EXIT_FAILURE;
  }
}
