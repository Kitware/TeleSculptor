/*ckwg +29
 * Copyright 2014-2020 by Kitware, Inc.
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
 * \brief Sparse Bungle Adjustment utility
 */

#include "tool_common.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <exception>
#include <string>
#include <vector>

#include <vital/config/config_block.h>
#include <vital/config/config_block_io.h>

#include <vital/algo/bundle_adjust.h>
#include <vital/algo/estimate_canonical_transform.h>
#include <vital/algo/estimate_similarity_transform.h>
#include <vital/algo/initialize_cameras_landmarks.h>
#include <vital/algo/filter_tracks.h>
#include <vital/algo/triangulate_landmarks.h>
#include <vital/algo/video_input.h>
#include <vital/exceptions.h>
#include <vital/io/camera_from_metadata.h>
#include <vital/io/eigen_io.h>
#include <vital/io/landmark_map_io.h>
#include <vital/io/metadata_io.h>
#include <vital/io/track_set_io.h>
#include <vital/plugin_loader/plugin_manager.h>
#include <vital/types/feature_track_set.h>
#include <vital/vital_types.h>
#include <vital/types/geodesy.h>
#include <vital/util/cpu_timer.h>
#include <vital/util/get_paths.h>

#include <kwiversys/SystemTools.hxx>
#include <kwiversys/CommandLineArguments.hxx>

#include <arrows/core/metrics.h>
#include <arrows/core/match_matrix.h>
#include <arrows/core/necker_reverse.h>
#include <arrows/core/transform.h>

#include <arrows/core/colorize.h>
#include <maptk/geo_reference_points_io.h>
#include <vital/types/local_geo_cs.h>
#include <maptk/version.h>

typedef kwiversys::SystemTools     ST;

static kwiver::vital::logger_handle_t main_logger( kwiver::vital::get_logger( "bundle_adjust_tracks_tool" ) );

static kwiver::vital::config_block_sptr default_config()
{

  kwiver::vital::config_block_sptr config = kwiver::vital::config_block::empty_config("bundle_adjust_tracks_tool");

  config->set_value("video_source", "",
                    "Path to an input file to be opened as a video. "
                    "This could be either a video file or a text file "
                    "containing new-line separated paths to sequential "
                    "image files.");

  config->set_value("input_track_file", "",
                    "Path an input file containing feature tracks");

  config->set_value("filtered_track_file", "",
                    "Path to write a file containing filtered feature tracks");

  config->set_value("init_cameras_with_metadata", false,
                    "Enables initialization of cameras from video metadata."
                    "\n"
                    "This is mutually exclusive with the input_krtd_files "
                    "option for system initialization, and shadowed by the "
                    "input_reference_points option when using an "
                    "st_estimator.");

  config->set_value("input_krtd_files", "",
                    "A directory containing input KRTD camera files.\n"
                    "\n"
                    "This is mutually exclusive with init_cameras_with_metadata "
                    "option for system initialization, and shadowed by the "
                    "input_reference_points_file when using an st_estimator.");

  config->set_value("input_reference_points_file", "",
                    "File containing reference points to use for reprojection "
                    "of results into the geographic coordinate system.\n"
                    "\n"
                    "This option is NOT mutually exclusive with input_*_files "
                    "options when using an st_estimator. When both this and "
                    "another input files option are specified, use of the "
                    "reference file is given priority over the input "
                    "cameras.\n"
                    "\n"
                    "Reference points file format (lm=landmark, tNsM=track N state M):\n"
                    "\tlm1.x lm1.y lm1.z t1s1.frame t1s1.x t1s1.y t1s2.frame t1s2.x t1s2.y ...\n"
                    "\tlm2.x lm2.y lm2.z t2s1.frame t2s1.x t2s1.y t2s2.frame t2s2.x t2s2.y ...\n"
                    "\t...\n"
                    "\n"
                    "At least 3 landmarks must be given, with at least 2 "
                    "track states recorded for each landmark, for "
                    "transformation estimation to converge, however more of "
                    "each is recommended.\n"
                    "\n"
                    "Landmark z position, or altitude, should be provided in meters.");

  config->set_value("initialize_unloaded_cameras", "true",
                    "When loading a subset of cameras, should we optimize only the "
                    "loaded cameras or also initialize and optimize the unspecified cameras");

  config->set_value("geo_origin_file", "output/geo_origin.txt",
                    "This file contains the geographical location of the origin "
                    "of the local cartesian coordinate system used in the camera "
                    "and landmark files.  This file is use for input and output. "
                    "If the files exists it will be read to define the origin. "
                    "If the file does not exist an origin will be computed from "
                    "geographic metadata provided and written to this file. "
                    "The file format is ASCII (degrees, meters):\n"
                    "latitude longitude altitude");

  config->set_value("output_ply_file", "output/landmarks.ply",
                    "Path to the output PLY file in which to write "
                    "resulting 3D landmark points");

  config->set_value("output_pos_dir", "output/pos",
                    "A directory in which to write the output POS files.");

  config->set_value("output_krtd_dir", "output/krtd",
                    "A directory in which to write the output KRTD files.");

  config->set_value("camera_sample_rate", "1",
                    "Sub-sample the cameras for by this rate.\n"
                    "Set to 1 to use all cameras, "
                    "2 to use every other camera, etc.");

  config->set_value("necker_reverse_input", "false",
                    "Apply a Necker reversal to the initial cameras and landmarks");

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

  config->set_value("ins:rotation_offset", "0 0 0 1",
                    "A quaternion used to offset rotation from metadata "
                    "when updating cameras. This option is only relevent if "
                    "init_cameras_with_metadata is enabled.");

  config->set_value("krtd_clean_up", "false",
                    "Delete all previously existing KRTD files present in output_krtd_dir before writing new KRTD files.");

  config->set_value("depthmaps_images_file", "",
                    "An optional file containing paths to depthmaps as image datas.");

  auto default_vi = kwiver::vital::algo::video_input::create("pos");
  kwiver::vital::algo::video_input::get_nested_algo_configuration("video_reader", config, default_vi);
  kwiver::vital::algo::filter_tracks::get_nested_algo_configuration("track_filter", config,
                                                     kwiver::vital::algo::filter_tracks_sptr());
  kwiver::vital::algo::bundle_adjust::get_nested_algo_configuration("bundle_adjuster", config,
                                                     kwiver::vital::algo::bundle_adjust_sptr());
  kwiver::vital::algo::initialize_cameras_landmarks
      ::get_nested_algo_configuration("initializer", config,
                                      kwiver::vital::algo::initialize_cameras_landmarks_sptr());
  kwiver::vital::algo::triangulate_landmarks::get_nested_algo_configuration("triangulator", config,
                        kwiver::vital::algo::triangulate_landmarks_sptr());
  kwiver::vital::algo::estimate_similarity_transform::get_nested_algo_configuration("st_estimator", config,
                                                                     kwiver::vital::algo::estimate_similarity_transform_sptr());
  kwiver::vital::algo::estimate_canonical_transform::get_nested_algo_configuration("can_tfm_estimator", config,
                                                                     kwiver::vital::algo::estimate_canonical_transform_sptr());

  return config;
}


// ------------------------------------------------------------------
static bool check_config(kwiver::vital::config_block_sptr config)
{
  bool config_valid = true;

#define MAPTK_CONFIG_FAIL(msg) \
  LOG_ERROR(main_logger, "Config Check Fail: " << msg); \
  config_valid = false

  if (!config->has_value("video_source"))
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

  if (!config->has_value("input_track_file"))
  {
    MAPTK_CONFIG_FAIL("Not given a tracks file path");
  }
  else if (! ST::FileExists( config->get_value<std::string>("input_track_file"), true ) )
  {
    MAPTK_CONFIG_FAIL("Given tracks file path doesn't point to an existing file.");
  }

  // Checking input cameras and reference points file existance.
  bool input_metadata = config->get_value<bool>("init_cameras_with_metadata", false);
  bool input_krtd = false;
  if (config->get_value<std::string>("input_krtd_files", "") != "")
  {
    input_krtd = true;
    if (! ST::FileExists(config->get_value<std::string>("input_krtd_files")))
    {
      MAPTK_CONFIG_FAIL("KRTD input path given, but does not point to an existing location.");
    }
  }
  if (input_metadata && input_krtd)
  {
    MAPTK_CONFIG_FAIL("Both input metadata and KRTD cameras were given. Don't know which to use!");
  }
  if (config->get_value<std::string>("input_reference_points_file", "") != "")
  {
    if (! ST::FileExists(config->get_value<std::string>("input_reference_points_file"), true ))
    {
      MAPTK_CONFIG_FAIL("Path given for input reference points file does not exist.");
    }
  }


  if (!kwiver::vital::algo::video_input::check_nested_algo_configuration("video_reader", config))
  {
    MAPTK_CONFIG_FAIL("video_reader configuration check failed");
  }
  if (!kwiver::vital::algo::filter_tracks::check_nested_algo_configuration("track_filter", config))
  {
    MAPTK_CONFIG_FAIL("Failed config check in track_filter algorithm.");
  }
  if (!kwiver::vital::algo::bundle_adjust::check_nested_algo_configuration("bundle_adjuster", config))
  {
    MAPTK_CONFIG_FAIL("Failed config check in bundle_adjuster algorithm.");
  }
  if (!kwiver::vital::algo::initialize_cameras_landmarks
            ::check_nested_algo_configuration("initializer", config))
  {
    MAPTK_CONFIG_FAIL("Failed config check in initializer algorithm.");
  }
  if (!kwiver::vital::algo::triangulate_landmarks::check_nested_algo_configuration("triangulator", config))
  {
    MAPTK_CONFIG_FAIL("Failed config check in triangulator algorithm.");
  }
  if (config->has_value("st_estimator:type") && config->get_value<std::string>("st_estimator:type") != "")
  {
    if (!kwiver::vital::algo::estimate_similarity_transform::check_nested_algo_configuration("st_estimator", config))
    {
      MAPTK_CONFIG_FAIL("Failed config check in st_estimator algorithm.");
    }
  }
  if (config->has_value("can_tfm_estimator:type") && config->get_value<std::string>("can_tfm_estimator:type") != "")
  {
    if (!kwiver::vital::algo::estimate_canonical_transform::check_nested_algo_configuration("can_tfm_estimator", config))
    {
      MAPTK_CONFIG_FAIL("Failed config check in can_tfm_estimator algorithm.");
    }
  }


#undef MAPTK_CONFIG_FAIL

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
/// Subsample a every Nth camera, where N is specfied by factor
/**
 * Uses camera frame numbers to determine subsample. This is fine when we
 * assume the cameras given are sequential and always start with frame 0.
 * This will behave in possibly undesired ways when the given cameras are not in
 * sequential frame order, or the first camera's frame is not a multiple of
 * \c factor.  This function ensures that the first and last cameras are
 * included
 */
kwiver::vital::camera_map_sptr
subsample_cameras(kwiver::vital::camera_map_sptr cameras, unsigned factor)
{
  kwiver::vital::camera_map::map_camera_t cams = cameras->cameras();
  kwiver::vital::camera_map::map_camera_t sub_cams;
  for(const kwiver::vital::camera_map::map_camera_t::value_type& p : cams)
  {
    if(p.first % factor == 0)
    {
      sub_cams.insert(p);
    }
  }
  // Also include the last camera
  sub_cams.insert(*cams.rbegin());
  return kwiver::vital::camera_map_sptr(new kwiver::vital::simple_camera_map(sub_cams));
}


// Generic configuration based input camera load function.
//
// The local_cs and input_cameras objects may or may not be updated based on
// configuration.
//
// Returns false if errors occurred, otherwise true. Returns true even if no
// input cameras loaded (check size of input_cameras structure).
bool load_input_cameras(kwiver::vital::config_block_sptr config,
                        std::map<kwiver::vital::frame_id_t, kwiver::vital::metadata_sptr> const& md_map,
                        std::map<kwiver::vital::frame_id_t, std::string> const& basename_map,
                        kwiver::vital::local_geo_cs & local_cs,
                        kwiver::vital::camera_map::map_camera_t & input_cameras)
{
  // configuration check assured mutual exclusivity
  if (config->get_value<bool>("init_cameras_with_metadata", false))
  {
    // create initial cameras from metadata
    kwiver::vital::simple_camera_perspective base_camera =
      base_camera_from_config(config->subblock("base_camera"));
    kwiver::vital::rotation_d ins_rot_offset =
      config->get_value<kwiver::vital::rotation_d>("ins:rotation_offset",
                                                   kwiver::vital::rotation_d());

    // create initial cameras from metadata
    input_cameras = kwiver::vital::initialize_cameras_with_metadata(
      md_map, base_camera, local_cs, true, ins_rot_offset);

    return !input_cameras.empty();
  }
  else if (config->get_value<std::string>("input_krtd_files", "") != "")
  {
    std::string krtd_dir = config->get_value<std::string>("input_krtd_files");
    input_cameras = kwiver::maptk::load_input_cameras_krtd(krtd_dir, basename_map);
    if (input_cameras.empty())
    {
      return false;
    }
  }

  // No input specified
  return true;
}


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

  // Tell PROJ where to find its data files
  std::string rel_proj_path = kwiver::vital::get_executable_path() + "/../share/proj";
  if ( kwiversys::SystemTools::FileExists(rel_proj_path) &&
       kwiversys::SystemTools::FileIsDirectory(rel_proj_path) )
  {
    kwiversys::SystemTools::PutEnv("PROJ_LIB="+rel_proj_path);
  }

  // Set config to algo chain
  // Get config from algo chain after set
  // Check config validity, store result
  //
  // If -o/--output-config given,
  //   output config result and notify of current (in)validity
  // Else error if provided config not valid.

  // Set up top level configuration w/ defaults where applicable.
  kwiver::vital::config_block_sptr config = kwiver::vital::config_block::empty_config();
  kwiver::vital::algo::video_input_sptr video_reader;
  kwiver::vital::algo::bundle_adjust_sptr bundle_adjuster;
  kwiver::vital::algo::initialize_cameras_landmarks_sptr initializer;
  kwiver::vital::algo::filter_tracks_sptr track_filter;
  kwiver::vital::algo::triangulate_landmarks_sptr triangulator;
  kwiver::vital::algo::estimate_similarity_transform_sptr st_estimator;
  kwiver::vital::algo::estimate_canonical_transform_sptr can_tfm_estimator;

  // If -c/--config given, read in confg file, merge in with default just generated
  if( ! opt_config.empty() )
  {
    const std::string prefix = kwiver::vital::get_executable_path() + "/..";
    config->merge_config(kwiver::vital::read_config_file(opt_config, "telesculptor",
                                                         TELESCULPTOR_VERSION, prefix));
  }


  kwiver::vital::algo::video_input::set_nested_algo_configuration("video_reader", config, video_reader);
  kwiver::vital::algo::bundle_adjust::set_nested_algo_configuration("bundle_adjuster", config, bundle_adjuster);
  kwiver::vital::algo::triangulate_landmarks::set_nested_algo_configuration("triangulator", config, triangulator);
  kwiver::vital::algo::initialize_cameras_landmarks::set_nested_algo_configuration("initializer", config, initializer);
  kwiver::vital::algo::filter_tracks::set_nested_algo_configuration("track_filter", config, track_filter);
  kwiver::vital::algo::estimate_similarity_transform::set_nested_algo_configuration("st_estimator", config, st_estimator);
  kwiver::vital::algo::estimate_canonical_transform::set_nested_algo_configuration("can_tfm_estimator", config, can_tfm_estimator);

  kwiver::vital::config_block_sptr dflt_config = default_config();
  dflt_config->merge_config(config);
  config = dflt_config;

  bool valid_config = check_config(config);

  if( ! opt_out_config.empty() )
  {
    kwiver::vital::algo::video_input::get_nested_algo_configuration("video_reader", config, video_reader);
    kwiver::vital::algo::bundle_adjust::get_nested_algo_configuration("bundle_adjuster", config, bundle_adjuster);
    kwiver::vital::algo::triangulate_landmarks::get_nested_algo_configuration("triangulator", config, triangulator);
    kwiver::vital::algo::initialize_cameras_landmarks::get_nested_algo_configuration("initializer", config, initializer);
    kwiver::vital::algo::filter_tracks::get_nested_algo_configuration("track_filter", config, track_filter);
    kwiver::vital::algo::estimate_similarity_transform::get_nested_algo_configuration("st_estimator", config, st_estimator);
    kwiver::vital::algo::estimate_canonical_transform::get_nested_algo_configuration("can_tfm_estimator", config, can_tfm_estimator);

    write_config_file(config, opt_out_config );
    if(valid_config)
    {
      LOG_INFO(main_logger, "Configuration file contained valid parameters"
                            << " and may be used for running");
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

  //
  // Read the track file
  //
  std::string track_file = config->get_value<std::string>("input_track_file");
  LOG_INFO(main_logger, "loading track file: " << track_file);
  kwiver::vital::feature_track_set_sptr tracks = kwiver::vital::read_feature_track_file(track_file);

  LOG_DEBUG(main_logger, "loaded "<<tracks->size()<<" tracks");
  if( tracks->size() == 0 )
  {
    LOG_ERROR(main_logger, "No tracks loaded.");
    return EXIT_FAILURE;
  }

  //
  // Filter the tracks
  //
  {
    kwiver::vital::scoped_cpu_timer t( "track filtering" );
    auto filt_tracks = track_filter->filter(tracks);
    tracks = std::static_pointer_cast<kwiver::vital::feature_track_set>(filt_tracks);
    LOG_DEBUG(main_logger, "filtered down to "<<tracks->size()<<" long tracks");

    // write out filtered tracks if output file is specified
    if (config->has_value("filtered_track_file"))
    {
      std::string out_track_file = config->get_value<std::string>("filtered_track_file");
      if( out_track_file != "" )
      {
        kwiver::vital::write_feature_track_file(tracks, out_track_file);
      }
    }

    if( tracks->size() == 0 )
    {
      LOG_ERROR(main_logger, "All track have been filtered. "
                             << "Try decreasing \"min_track_len\" "
                             << "or \"min_mm_importance\"");
      return EXIT_FAILURE;
    }
  }

  //
  // Read the Video (metadata only, no pixels)
  //
  std::map<kwiver::vital::frame_id_t, kwiver::vital::metadata_sptr> md_map;
  std::map<kwiver::vital::frame_id_t, std::string> basename_map;

  LOG_INFO( main_logger, "Reading Video" );
  std::string video_source = config->get_value<std::string>("video_source");
  video_reader->open(video_source);

  kwiver::vital::timestamp ts;
  while( video_reader->next_frame(ts) )
  {
    auto md_vec = video_reader->frame_metadata();
    if( md_vec.empty() || !md_vec[0] )
    {
      continue;
    }
    auto md = md_vec[0];
    auto frame = ts.get_frame();
    md_map[frame] = md;
    std::string basename = kwiver::vital::basename_from_metadata(md, frame);
    basename_map[frame] = basename;
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


  //
  // Initialize input and main cameras
  //
  // Initialize input camera map based on which input files were given, if any.
  // If input_cameras is empty after this method, then there were no input
  // camera files.
  //
  // Config check above ensures validity + mutual exclusivity of these options
  kwiver::vital::camera_map::map_camera_t input_cameras;
  if (!load_input_cameras(config, md_map, basename_map, local_cs, input_cameras))
  {
    LOG_ERROR(main_logger, "Failed to load input cameras");
    return EXIT_FAILURE;
  }

  // Copy input cameras into main camera map
  kwiver::vital::camera_map::map_camera_t cameras;
  kwiver::vital::landmark_map_sptr lm_map;
  kwiver::vital::camera_map_sptr input_cam_map(new kwiver::vital::simple_camera_map(input_cameras));
  if (input_cameras.size() != 0)
  {
    for(kwiver::vital::camera_map::map_camera_t::value_type &v : input_cameras)
    {
      cameras[v.first] = v.second->clone();
    }
  }
  kwiver::vital::camera_map_sptr cam_map;
  if(!cameras.empty())
  {
    cam_map = kwiver::vital::camera_map_sptr(new kwiver::vital::simple_camera_map(cameras));
  }

  kwiver::vital::landmark_map_sptr reference_landmarks(new kwiver::vital::simple_landmark_map());
  kwiver::vital::feature_track_set_sptr reference_tracks = std::make_shared<kwiver::vital::feature_track_set>();
  if (config->get_value<std::string>("input_reference_points_file", "") != "")
  {
    kwiver::vital::path_t ref_file = config->get_value<kwiver::vital::path_t>("input_reference_points_file");

    // Load up landmarks and assocaited tracks from file, (re)initializing
    // local coordinate system object to the reference.
    kwiver::maptk::load_reference_file(ref_file, local_cs, reference_landmarks, reference_tracks);
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

  // apply necker reversal if requested
  bool necker_reverse_input = config->get_value<bool>("necker_reverse_input", false);
  if (necker_reverse_input)
  {
    LOG_INFO(main_logger, "Applying Necker reversal");
    kwiver::arrows::core::necker_reverse(cam_map, lm_map);
  }

  bool init_unloaded_cams = config->get_value<bool>("initialize_unloaded_cameras", true);
  if (init_unloaded_cams)
  {
    if( cam_map )
    {
      cameras = cam_map->cameras();
    }
    for(const kwiver::vital::frame_id_t& id : tracks->all_frame_ids())
    {
      // if id is already in the map, do nothing.
      // if id is not it the map add a null camera pointer
      cameras[id];
    }
    cam_map = kwiver::vital::camera_map_sptr(new kwiver::vital::simple_camera_map(cameras));
  }

  //
  // Cut down input cameras if a sub-sample rate was specified
  //
  unsigned int cam_samp_rate = config->get_value<unsigned int>("camera_sample_rate");
  if(cam_samp_rate > 1)
  {
    kwiver::vital::scoped_cpu_timer t( "Tool-level sub-sampling" );

    // If there are no cameras loaded, create a map of NULL cameras to subsample
    if( !cam_map )
    {
      for(const kwiver::vital::frame_id_t& id : tracks->all_frame_ids())
      {
        cameras[id] = kwiver::vital::camera_sptr();
      }
      cam_map = kwiver::vital::camera_map_sptr(new kwiver::vital::simple_camera_map(cameras));
    }

    kwiver::vital::camera_map_sptr subsampled_cams = subsample_cameras(cam_map, cam_samp_rate);

    // If we were given reference landmarks and tracks, make sure to include
    // the cameras for frames reference track states land on. Required for
    // sba-space landmark triangulation and correlation later.
    if (reference_tracks->size() > 0)
    {
      kwiver::vital::camera_map::map_camera_t cams = cam_map->cameras(),
                                      sub_cams = subsampled_cams->cameras();
      // for each track state in each reference track, make sure that the
      // state's frame's camera is in the sub-sampled set of cameras
      for(kwiver::vital::track_sptr const t : reference_tracks->tracks())
      {
        for (auto ts : *t)
        {
          if (cams.count(ts->frame()) > 0)
          {
            sub_cams.insert(*cams.find(ts->frame()));
          }
        }
      }
      subsampled_cams = kwiver::vital::camera_map_sptr(new kwiver::vital::simple_camera_map(sub_cams));
    }

    cam_map = subsampled_cams;
    LOG_INFO(main_logger, "Subsampled down to "<<cam_map->size()<<" cameras");
  }

  //
  // Initialize cameras and landmarks
  //
  {
    kwiver::vital::scoped_cpu_timer t( "Initializing cameras and landmarks" );
    initializer->initialize(cam_map, lm_map, tracks);
  }

  //
  // Run bundle adjustment
  //
  { // scope block
    kwiver::vital::scoped_cpu_timer t( "Tool-level SBA algorithm" );

    double init_rmse = kwiver::arrows::reprojection_rmse(cam_map->cameras(),
                                                        lm_map->landmarks(),
                                                        tracks->tracks());
    LOG_DEBUG(main_logger, "initial reprojection RMSE: " << init_rmse);

    bundle_adjuster->optimize(cam_map, lm_map, tracks);

    double end_rmse = kwiver::arrows::reprojection_rmse(cam_map->cameras(),
                                                       lm_map->landmarks(),
                                                       tracks->tracks());
    LOG_DEBUG(main_logger, "final reprojection RMSE: " << end_rmse);
  }


  //
  // Adjust cameras/landmarks based on input cameras/reference points
  //
  // If we were given POS files / reference points as input, compute a
  // similarity transform from the refined cameras to the POS file / reference
  // point structures. Then, apply the estimated transform to the refined
  // camera positions and landmarks.
  //
  // The effect of this is to put the refined cameras and landmarks into the
  // same coordinate system as the input cameras / reference points.
  //
  if (st_estimator || can_tfm_estimator)
  {
    kwiver::vital::scoped_cpu_timer t_1( "--> st estimation and application" );
    LOG_INFO(main_logger, "Estimating similarity transform from post-SBA to original space");

    // initialize identity transform
    kwiver::vital::similarity_d sim_transform;

    // Prioritize use of reference landmarks/tracks over use of POS files for
    // transformation out of SBA-space.
    if (reference_landmarks->size() > 0 && reference_tracks->size() > 0)
    {
      kwiver::vital::scoped_cpu_timer t_2( "similarity transform estimation from ref file" );
      LOG_INFO(main_logger, "Using reference landmarks/tracks");

      // Generate corresponding landmarks in SBA-space based on transformed
      //    cameras and reference landmarks/tracks via triangulation.
      LOG_INFO(main_logger, "Triangulating SBA-space reference landmarks from "
                            << "reference tracks and post-SBA cameras");
      kwiver::vital::landmark_map_sptr sba_space_landmarks(new kwiver::vital::simple_landmark_map(reference_landmarks->landmarks()));
      triangulator->triangulate(cam_map, reference_tracks, sba_space_landmarks);
      if (sba_space_landmarks->size() < reference_landmarks->size())
      {
        LOG_WARN(main_logger, "Only " << sba_space_landmarks->size()
                              << " out of " << reference_landmarks->size()
                              << " reference points triangulated");
      }

      double post_tri_rmse = kwiver::arrows::reprojection_rmse(cam_map->cameras(),
                                                              sba_space_landmarks->landmarks(),
                                                              reference_tracks->tracks());
      LOG_DEBUG(main_logger, "Post-triangulation RMSE: " << post_tri_rmse);

      // Estimate ST from sba-space to reference space.
      LOG_INFO(main_logger, "Estimating transform to reference landmarks (from "
                            << "SBA-space ref landmarks)");
      sim_transform = st_estimator->estimate_transform(sba_space_landmarks, reference_landmarks);
    }
    else if (st_estimator && input_cam_map->size() > 0)
    {
      kwiver::vital::scoped_cpu_timer t_2( "    similarity transform estimation from camera" );

      LOG_INFO(main_logger, "Estimating transform to refined cameras "
                            << "(from input cameras)");
      sim_transform = st_estimator->estimate_transform(cam_map, input_cam_map);
    }
    else if (can_tfm_estimator)
    {
      // In the absence of other information, use a canonical transformation
      sim_transform = can_tfm_estimator->estimate_transform(cam_map, lm_map);
    }

    LOG_DEBUG(main_logger, "Estimated Transformation: " << sim_transform);

    // apply to cameras and landmarks
    LOG_INFO(main_logger, "Applying transform to cameras and landmarks");
    cam_map = kwiver::arrows::core::transform(cam_map, sim_transform);
    lm_map = kwiver::arrows::core::transform(lm_map, sim_transform);
  }

  //
  // Compute landmark colors
  //
  lm_map = kwiver::arrows::core::compute_landmark_colors(*lm_map, *tracks);

  //
  // Write the output PLY file
  //
  if( config->has_value("output_ply_file") )
  {
    kwiver::vital::scoped_cpu_timer t( "writing output PLY file" );
    std::string ply_file = config->get_value<std::string>("output_ply_file");
    write_ply_file(lm_map, ply_file);
  }

  //
  // Write the output POS files
  //
  if( config->has_value("output_pos_dir") )
  {
    LOG_INFO(main_logger, "Writing output POS files");
    kwiver::vital::scoped_cpu_timer t( "--> Writing output POS files" );

    kwiver::vital::path_t pos_dir = config->get_value<std::string>("output_pos_dir");
    // Create updated metadata from adjusted cameras for POS file output.
    typedef std::map<kwiver::vital::frame_id_t, kwiver::vital::metadata_sptr> md_map_t;
    md_map_t updated_md_map;
    update_metadata_from_cameras(cam_map->cameras(), local_cs, updated_md_map);
    for(auto const& p : updated_md_map)
    {
      if (p.second)
      {
        kwiver::vital::path_t out_pos_file = pos_dir + "/" + basename_map[p.first] + ".pos";
        kwiver::vital::write_pos_file( *p.second, out_pos_file);
      }
    }
    if (updated_md_map.size() == 0)
    {
      LOG_WARN(main_logger, "INS map empty, no output POS files written");
    }
  }

  //
  // Write the output KRTD files
  //
  if (config->get_value<bool>("krtd_clean_up") )
  {

    LOG_INFO(main_logger, "Cleaning "
                          << config->get_value<std::string>("output_krtd_dir")
                          << " before writing new files.");

    kwiver::vital::path_t krtd_dir = config->get_value<std::string>("output_krtd_dir");
    std::vector<kwiver::vital::path_t> files = kwiver::maptk::files_in_dir(krtd_dir);

    for (size_t i = 0; i < files.size(); ++i)
    {
      if (ST::GetFilenameLastExtension(files[i]) == ".krtd")
      {
        ST::RemoveFile(files[i]);
      }
    }

  }

  if( config->has_value("output_krtd_dir") )
  {
    LOG_INFO(main_logger, "Writing output KRTD files");
    kwiver::vital::scoped_cpu_timer t("--> Writing output KRTD files" );

    kwiver::vital::path_t krtd_dir = config->get_value<std::string>("output_krtd_dir");
    typedef kwiver::vital::camera_map::map_camera_t::value_type cam_map_val_t;
    for(cam_map_val_t const& p : cam_map->cameras())
    {
      if (p.second)
      {
        kwiver::vital::path_t out_krtd_file = krtd_dir + "/" + basename_map[p.first] + ".krtd";
        auto cam_ptr = std::dynamic_pointer_cast<kwiver::vital::camera_perspective>( p.second );
        write_krtd_file( *cam_ptr, out_krtd_file );
      }
    }
  }

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
    LOG_ERROR(main_logger, "Exception caught: " << e.what());

    return EXIT_FAILURE;
  }
  catch (...)
  {
    LOG_ERROR(main_logger, "Unknown exception caught");

    return EXIT_FAILURE;
  }
}
