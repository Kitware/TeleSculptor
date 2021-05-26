// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

/**
 * \file
 * \brief Feature tracker utility
 */

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
#include <vital/io/track_set_io.h>
#include <vital/vital_types.h>
#include <vital/algo/image_io.h>
#include <vital/algo/convert_image.h>
#include <vital/algo/track_features.h>
#include <vital/algo/compute_ref_homography.h>
#include <vital/algo/video_input.h>
#include <vital/plugin_loader/plugin_manager.h>
#include <vital/util/get_paths.h>
#include <vital/util/transform_image.h>

#include <kwiversys/SystemTools.hxx>
#include <kwiversys/CommandLineArguments.hxx>

#include <maptk/version.h>

typedef kwiversys::SystemTools ST;
typedef kwiversys::CommandLineArguments argT;

static kwiver::vital::logger_handle_t main_logger( kwiver::vital::get_logger( "track_features_tool" ) );

// ------------------------------------------------------------------
static kwiver::vital::config_block_sptr default_config()
{
  kwiver::vital::config_block_sptr config = kwiver::vital::config_block::empty_config("feature_tracker_tool");

  config->set_value("video_source", "",
                    "Path to an input file to be opened as a video. "
                    "This could be either a video file or a text file "
                    "containing new-line separated paths to sequential "
                    "image files.");
  config->set_value("mask_list_file", "",
                    "Optional path to an input file containing new-line "
                    "separated paths to mask images. This list should be "
                    "parallel in association to frames provided by the "
                    "``video_source`` video. Mask images must be the same size "
                    "as the image they are associated with.\n"
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

  kwiver::vital::algo::video_input::get_nested_algo_configuration("video_reader", config,
                                      kwiver::vital::algo::video_input_sptr());
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

// ------------------------------------------------------------------
static bool check_config(kwiver::vital::config_block_sptr config)
{
  bool config_valid = true;

#define MAPTK_CONFIG_FAIL(msg) \
  LOG_ERROR(main_logger, "Config Check Fail: " << msg); \
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
              ! ST::FileIsDirectory( ST::GetFilenamePath( fp ) ))
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

  if (!config->has_value("output_tracks_file") ||
      config->get_value<std::string>("output_tracks_file") == "" )
  {
    MAPTK_CONFIG_FAIL("Config needs value output_tracks_file");
  }
  else if ( ! ST::FileIsDirectory( ST::CollapseFullPath( ST::GetFilenamePath(
              config->get_value<kwiver::vital::path_t>("output_tracks_file") ) ) ) )
  {
    MAPTK_CONFIG_FAIL("output_tracks_file is not in a valid directory");
  }

  if (!kwiver::vital::algo::video_input::check_nested_algo_configuration("video_reader", config))
  {
    MAPTK_CONFIG_FAIL("video_reader configuration check failed");
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
  kwiver::vital::algo::track_features_sptr feature_tracker;
  kwiver::vital::algo::image_io_sptr image_reader;
  kwiver::vital::algo::convert_image_sptr image_converter;
  kwiver::vital::algo::compute_ref_homography_sptr out_homog_generator;

  // If -c/--config given, read in confg file, merge in with default just generated
  if( ! opt_config.empty() )
  {
    const std::string prefix = kwiver::vital::get_executable_path() + "/..";
    config->merge_config(kwiver::vital::read_config_file(opt_config, "telesculptor",
                                                         TELESCULPTOR_VERSION, prefix));
  }

  kwiver::vital::algo::video_input::set_nested_algo_configuration("video_reader", config, video_reader);
  kwiver::vital::algo::video_input::get_nested_algo_configuration("video_reader", config, video_reader);
  kwiver::vital::algo::track_features::set_nested_algo_configuration("feature_tracker", config, feature_tracker);
  kwiver::vital::algo::track_features::get_nested_algo_configuration("feature_tracker", config, feature_tracker);
  kwiver::vital::algo::image_io::set_nested_algo_configuration("image_reader", config, image_reader);
  kwiver::vital::algo::image_io::get_nested_algo_configuration("image_reader", config, image_reader);
  kwiver::vital::algo::convert_image::set_nested_algo_configuration("convert_image", config, image_converter);
  kwiver::vital::algo::convert_image::get_nested_algo_configuration("convert_image", config, image_converter);
  kwiver::vital::algo::compute_ref_homography::set_nested_algo_configuration("output_homography_generator", config, out_homog_generator);
  kwiver::vital::algo::compute_ref_homography::get_nested_algo_configuration("output_homography_generator", config, out_homog_generator);

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
  std::string output_tracks_file = config->get_value<std::string>("output_tracks_file");

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

  // verify that we can open the output file for writing
  // so that we don't find a problem only after spending
  // hours of computation time.
  std::ofstream ofs(output_tracks_file.c_str());
  if (!ofs)
  {
    LOG_ERROR(main_logger, "Could not open track file for writing: \""
                           << output_tracks_file << "\"");
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
      LOG_ERROR(main_logger, "Could not open homography file for writing: "
                             << homog_fp);
      return EXIT_FAILURE;
    }
  }

  // Track features on each frame sequentially
  kwiver::vital::feature_track_set_sptr tracks;
  while( video_reader->next_frame(ts) )
  {
    LOG_INFO(main_logger, "processing frame "<<ts.get_frame() );

    auto const image = video_reader->frame_image();
    auto const mdv = video_reader->frame_metadata();
    auto converted_image = image_converter->convert( image );
    if( !mdv.empty() )
    {
      converted_image->set_metadata( mdv[0] );
    }

    // Load the mask for this image if we were given a mask image list
    kwiver::vital::image_container_sptr mask, converted_mask;
    if( use_masks )
    {
      mask = image_reader->load( mask_files[ts.get_frame()] );

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
        kwiver::vital::image_of<bool> mask_image;
        kwiver::vital::cast_image( mask->get_image(), mask_image );
        kwiver::vital::transform_image( mask_image, [] (bool b) { return !b; } );
        LOG_DEBUG( main_logger,
                   "Inverting mask image pixels -- Done" );
        mask = std::make_shared<kwiver::vital::simple_image_container>( mask_image );
      }

      converted_mask = image_converter->convert( mask );
    }

    tracks = feature_tracker->track(tracks, ts.get_frame(),
                                    converted_image, converted_mask);
    if (tracks)
    {
      tracks = kwiver::arrows::core::extract_feature_colors(tracks, *image, ts.get_frame());
    }

    // Compute ref homography for current frame with current track set + write to file
    // -> still doesn't take into account a full shotbreak, which would incur a track reset
    if ( homog_ofs.is_open() )
    {
      LOG_DEBUG(main_logger, "writing homography");
      homog_ofs << *(out_homog_generator->estimate(ts.get_frame(), tracks)) << std::endl;
    }
  }

  if ( homog_ofs.is_open() )
  {
    homog_ofs.close();
  }

  // Writing out tracks to file
  kwiver::vital::write_feature_track_file(tracks, output_tracks_file);

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
