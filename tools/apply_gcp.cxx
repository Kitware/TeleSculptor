// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

/**
 * \file
 * \brief Apply Ground Control Points utility
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

#include <vital/algo/estimate_canonical_transform.h>
#include <vital/algo/estimate_similarity_transform.h>
#include <vital/algo/triangulate_landmarks.h>
#include <vital/algo/video_input.h>
#include <vital/exceptions.h>
#include <vital/io/camera_io.h>
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

#include <arrows/mvg/metrics.h>
#include <arrows/mvg/transform.h>

#include <maptk/geo_reference_points_io.h>
#include <vital/types/local_geo_cs.h>
#include <maptk/version.h>

typedef kwiversys::SystemTools     ST;

static kwiver::vital::logger_handle_t main_logger( kwiver::vital::get_logger( "apply_gcp_tool" ) );

static kwiver::vital::config_block_sptr default_config()
{

  kwiver::vital::config_block_sptr config = kwiver::vital::config_block::empty_config("apply_gcp_tool");

  config->set_value("video_source", "",
                    "Path to an input file to be opened as a video. "
                    "This could be either a video file or a text file "
                    "containing new-line separated paths to sequential "
                    "image files.");

  config->set_value("input_ply_file", "",
                    "Path to the PLY file from which to read 3D landmark points");

  config->set_value("input_krtd_files", "",
                    "A directory containing input KRTD camera files, or a text "
                    "file containing a newline-separated list of KRTD files.\n"
                    "\n"
                    "This is optional, leave blank to ignore.");

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

  auto default_vi = kwiver::vital::algo::video_input::create("image_list");
  kwiver::vital::algo::video_input::get_nested_algo_configuration("video_reader", config, default_vi);
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

  // Checking input cameras and reference points file existance.
  if (config->get_value<std::string>("input_krtd_files", "") != "")
  {
    if (! ST::FileExists(config->get_value<std::string>("input_krtd_files")))
    {
      MAPTK_CONFIG_FAIL("KRTD input path given, but does not point to an existing location.");
    }
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

  if( kwiver::vital::get_geo_conv() == nullptr )
  {
    std::cerr << "No geographic conversion module available" << std::endl;
    return EXIT_FAILURE;
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
  kwiver::vital::algo::triangulate_landmarks::set_nested_algo_configuration("triangulator", config, triangulator);
  kwiver::vital::algo::estimate_similarity_transform::set_nested_algo_configuration("st_estimator", config, st_estimator);
  kwiver::vital::algo::estimate_canonical_transform::set_nested_algo_configuration("can_tfm_estimator", config, can_tfm_estimator);

  kwiver::vital::config_block_sptr dflt_config = default_config();
  dflt_config->merge_config(config);
  config = dflt_config;

  bool valid_config = check_config(config);

  if( ! opt_out_config.empty() )
  {
    kwiver::vital::algo::video_input::get_nested_algo_configuration("video_reader", config, video_reader);
    kwiver::vital::algo::triangulate_landmarks::get_nested_algo_configuration("triangulator", config, triangulator);
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
  // Load Cameras and Landmarks
  //
  std::string krtd_dir = config->get_value<std::string>("input_krtd_files");
  kwiver::vital::camera_map::map_camera_t input_cameras =
    kwiver::maptk::load_input_cameras_krtd(krtd_dir, basename_map);
  if (input_cameras.empty())
  {
    LOG_ERROR(main_logger, "Failed to load input cameras");
    return EXIT_FAILURE;
  }

  kwiver::vital::landmark_map_sptr lm_map;
  if( config->has_value("input_ply_file") )
  {
    std::string ply_file = config->get_value<std::string>("input_ply_file");
    lm_map = kwiver::vital::read_ply_file(ply_file);
  }

  // Copy input cameras into main camera map
  kwiver::vital::camera_map::map_camera_t cameras;
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

      double post_tri_rmse =
        kwiver::arrows::mvg::reprojection_rmse(cam_map->cameras(),
                                               sba_space_landmarks->landmarks(),
                                               reference_tracks->tracks());
      LOG_DEBUG(main_logger, "Post-triangulation RMSE: " << post_tri_rmse);

      // Estimate ST from sba-space to reference space.
      LOG_INFO(main_logger, "Estimating transform to reference landmarks (from "
                            << "SBA-space ref landmarks)");
      sim_transform = st_estimator->estimate_transform(sba_space_landmarks, reference_landmarks);
    }
    else if (can_tfm_estimator)
    {
      // In the absence of other information, use a canonical transformation
      sim_transform = can_tfm_estimator->estimate_transform(cam_map, lm_map);
    }

    LOG_DEBUG(main_logger, "Estimated Transformation: " << sim_transform);

    // apply to cameras and landmarks
    LOG_INFO(main_logger, "Applying transform to cameras and landmarks");
    cam_map = kwiver::arrows::mvg::transform(cam_map, sim_transform);
    lm_map = kwiver::arrows::mvg::transform(lm_map, sim_transform);
  }

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
