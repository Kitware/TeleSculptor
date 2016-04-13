/*ckwg +29
 * Copyright 2014-2016 by Kitware, Inc.
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

#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <exception>
#include <string>
#include <vector>

#include <vital/vital_foreach.h>
#include <vital/config/config_block.h>
#include <vital/config/config_block_io.h>

#include <vital/algo/bundle_adjust.h>
#include <vital/algo/estimate_similarity_transform.h>
#include <vital/algo/geo_map.h>
#include <vital/algo/initialize_cameras_landmarks.h>
#include <vital/algo/triangulate_landmarks.h>
#include <vital/algorithm_plugin_manager.h>
#include <vital/exceptions.h>
#include <vital/io/camera_io.h>
#include <vital/io/eigen_io.h>
#include <vital/io/landmark_map_io.h>
#include <vital/io/track_set_io.h>
#include <vital/types/track_set.h>
#include <vital/vital_types.h>
#include <vital/util/cpu_timer.h>
#include <vital/util/get_paths.h>

#include <kwiversys/SystemTools.hxx>
#include <kwiversys/CommandLineArguments.hxx>
#include <kwiversys/Directory.hxx>

#include <maptk/colorize.h>
#include <maptk/ins_data_io.h>
#include <maptk/local_geo_cs.h>
#include <maptk/geo_reference_points_io.h>
#include <maptk/metrics.h>
#include <maptk/transform.h>
#include <maptk/version.h>

typedef kwiversys::SystemTools     ST;

static kwiver::vital::config_block_sptr default_config()
{

  kwiver::vital::config_block_sptr config = kwiver::vital::config_block::empty_config("bundle_adjust_tracks_tool");

  config->set_value("input_track_file", "",
                    "Path an input file containing feature tracks");

  config->set_value("filtered_track_file", "",
                    "Path to write a file containing filtered feature tracks");

  config->set_value("image_list_file", "",
                    "Path to the input image list file used to generated the "
                    "input tracks.");

  config->set_value("input_pos_files", "",
                    "A directory containing the input POS files, or a text file"
                    "containing a newline-separated list of POS files.\n"
                    "\n"
                    "This is optional, leave blank to ignore.\n"
                    "\n"
                    "This is mutually exclusive with the input_krtd_files "
                    "option for system initialization, and shadowed by the "
                    "input_reference_points option when using an "
                    "st_estimator.");

  config->set_value("input_krtd_files", "",
                    "A directory containing input KRTD camera files, or a text "
                    "file containing a newline-separated list of KRTD files.\n"
                    "\n"
                    "This is optional, leave blank to ignore.\n"
                    "\n"
                    "This is mutually exclusive with input_pos_files option "
                    "for system initialization, and shadowed by the "
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

  config->set_value("output_ply_file", "output/landmarks.ply",
                    "Path to the output PLY file in which to write "
                    "resulting 3D landmark points");

  config->set_value("output_pos_dir", "output/pos",
                    "A directory in which to write the output POS files.");

  config->set_value("output_krtd_dir", "output/krtd",
                    "A directory in which to write the output KRTD files.");

  config->set_value("min_track_length", "50",
                    "Filter the input tracks keeping those covering "
                    "at least this many frames.");

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
                    "A quaternion used to offset rotation data from POS files "
                    "when updating cameras. This option is only relevent if a "
                    "value is give to the input_pos_files option.");

  config->set_value("depthmaps_points_file", " ",
                    "An optional file containing paths to depthmaps as polydatas.");

  config->set_value("depthmaps_surfaces_file", " ",
                    "An optional file containing paths to depthmaps as structured grid.");

  kwiver::vital::algo::bundle_adjust::get_nested_algo_configuration("bundle_adjuster", config,
                                                     kwiver::vital::algo::bundle_adjust_sptr());
  kwiver::vital::algo::initialize_cameras_landmarks
      ::get_nested_algo_configuration("initializer", config,
                                      kwiver::vital::algo::initialize_cameras_landmarks_sptr());
  kwiver::vital::algo::triangulate_landmarks::get_nested_algo_configuration("triangulator", config,
                        kwiver::vital::algo::triangulate_landmarks_sptr());
  kwiver::vital::algo::geo_map::get_nested_algo_configuration("geo_mapper", config,
                                               kwiver::vital::algo::geo_map_sptr());
  kwiver::vital::algo::estimate_similarity_transform::get_nested_algo_configuration("st_estimator", config,
                                                                     kwiver::vital::algo::estimate_similarity_transform_sptr());

  return config;
}


// ------------------------------------------------------------------
static bool check_config(kwiver::vital::config_block_sptr config)
{
  bool config_valid = true;

#define MAPTK_CONFIG_FAIL(msg) \
  std::cerr << "Config Check Fail: " << msg << std::endl; \
  config_valid = false

  if (!config->has_value("input_track_file"))
  {
    MAPTK_CONFIG_FAIL("Not given a tracks file path");
  }
  else if (! ST::FileExists( config->get_value<std::string>("input_track_file"), true ) )
  {
    MAPTK_CONFIG_FAIL("Given tracks file path doesn't point to an existing file.");
  }

  if (!config->has_value("image_list_file"))
  {
    MAPTK_CONFIG_FAIL("Not given an image list file");
  }
  else if (! ST::FileExists( config->get_value<std::string>("image_list_file"), true ) )
  {
    MAPTK_CONFIG_FAIL("Given image list file path doesn't point to an existing file.");
  }

  // Checking input cameras and reference points file existance.
  bool input_pos = false,
       input_krtd = false;
  if (config->get_value<std::string>("input_pos_files", "") != "")
  {
    input_pos = true;
    if (! ST::FileExists(config->get_value<std::string>("input_pos_files")))
    {
      MAPTK_CONFIG_FAIL("POS input path given, but doesn't point to an existing location.");
    }
  }
  if (config->get_value<std::string>("input_krtd_files", "") != "")
  {
    input_krtd = true;
    if (! ST::FileExists(config->get_value<std::string>("input_krtd_files")))
    {
      MAPTK_CONFIG_FAIL("KRTD input path given, but does not point to an existing location.");
    }
  }
  if (input_pos && input_krtd)
  {
    MAPTK_CONFIG_FAIL("Both input POS and KRTD cameras were given. Don't know which to use!");
  }
  if (config->get_value<std::string>("input_reference_points_file", "") != "")
  {
    if (! ST::FileExists(config->get_value<std::string>("input_reference_points_file"), true ))
    {
      MAPTK_CONFIG_FAIL("Path given for input reference points file does not exist.");
    }
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
  if (!kwiver::vital::algo::geo_map::check_nested_algo_configuration("geo_mapper", config))
  {
    MAPTK_CONFIG_FAIL("Failed config check in geo_mapper algorithm.");
  }
  if (config->has_value("st_estimator:type") && config->get_value<std::string>("st_estimator:type") != "")
  {
    if (!kwiver::vital::algo::estimate_similarity_transform::check_nested_algo_configuration("st_estimator", config))
    {
      MAPTK_CONFIG_FAIL("Failed config check in st_estimator algorithm.");
    }
  }

#undef MAPTK_CONFIG_FAIL

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
/// filter track set by removing short tracks
kwiver::vital::track_set_sptr
filter_tracks(kwiver::vital::track_set_sptr tracks, size_t min_length)
{
  std::vector<kwiver::vital::track_sptr> trks = tracks->tracks();
  std::vector<kwiver::vital::track_sptr> good_trks;
  VITAL_FOREACH(kwiver::vital::track_sptr t, trks)
  {
    if( t->size() >= min_length )
    {
      good_trks.push_back(t);
    }
  }
  return kwiver::vital::track_set_sptr(new kwiver::vital::simple_track_set(good_trks));
}


// ------------------------------------------------------------------
/// Subsample a every Nth camera, where N is specfied by factor
/**
 * Uses camera frame numbers to determine subsample. This is fine when we
 * assume the cameras given are sequential and always start with frame 0.
 * This will behave in possibly undesired ways when the given cameras are not in
 * sequential frame order, or the first camera's frame is not a multiple of
 * \c factor.
 */
kwiver::vital::camera_map_sptr
subsample_cameras(kwiver::vital::camera_map_sptr cameras, unsigned factor)
{
  kwiver::vital::camera_map::map_camera_t cams = cameras->cameras();
  kwiver::vital::camera_map::map_camera_t sub_cams;
  VITAL_FOREACH(const kwiver::vital::camera_map::map_camera_t::value_type& p, cams)
  {
    if(p.first % factor == 0)
    {
      sub_cams.insert(p);
    }
  }
  return kwiver::vital::camera_map_sptr(new kwiver::vital::simple_camera_map(sub_cams));
}


// ------------------------------------------------------------------
// return a sorted list of files in a directory
std::vector< kwiver::vital::path_t >
files_in_dir(kwiver::vital::path_t const& vdir)
{
  std::vector< kwiver::vital::path_t > files;

  kwiversys::Directory dir;
  if ( 0 == dir.Load( vdir ) )
  {
    std::cerr << "Could not access directory \"" << vdir << std::endl;
    return files;
  }

  unsigned long num_files = dir.GetNumberOfFiles();
  for ( unsigned long i = 0; i < num_files; i++)
  {
    files.push_back( vdir + '/' + dir.GetFile( i ) );
  }

  std::sort( files.begin(), files.end() );
  return files;
}


// ------------------------------------------------------------------
/// Return a list of file paths either from a directory of files or from a
/// list of file paths
///
/// Returns false if we were given a file list and the file could not be
/// opened. Otherwise returns true.
bool
resolve_files(kwiver::vital::path_t const &p, std::vector< kwiver::vital::path_t > &files)
{
  if ( ST::FileIsDirectory( p) )
  {
    files = files_in_dir(p);
  }
  else
  {
    std::ifstream ifs(p.c_str());
    if (!ifs)
    {
      return false;
    }
    for (std::string line; std::getline(ifs, line);)
    {
      files.push_back(line);
    }
  }
  return true;
}


// Load input POS cameras from file, matching against the given image filename
// map, and updated local_cs and input_cameras structures.
//
// Returns false if a failure occurred
bool
load_input_cameras_pos(kwiver::vital::config_block_sptr config,
                       std::map<std::string, kwiver::vital::frame_id_t> const& filename2frame,
                       kwiver::maptk::local_geo_cs & local_cs,
                       kwiver::vital::camera_map::map_camera_t & input_cameras)
{
  kwiver::vital::scoped_cpu_timer t( "Initializing cameras from POS files" );

  std::string pos_files = config->get_value<std::string>("input_pos_files");
  std::vector< kwiver::vital::path_t > files;
  if (!resolve_files(pos_files, files))
  {
    std::cerr << "ERROR: Could not open POS file list." << std::endl;
    return false;
  }

  std::cerr << "loading POS files" <<std::endl;
  // Associating POS file to frame ID based on whether its filename stem is
  // the same as an image in the given image list (map created above).
  std::map<kwiver::vital::frame_id_t, kwiver::maptk::ins_data> ins_map;
  std::map<std::string, kwiver::vital::frame_id_t>::const_iterator it;
  VITAL_FOREACH(kwiver::vital::path_t const& fpath, files)
  {
    std::string pos_file_stem = ST::GetFilenameWithoutLastExtension( fpath );
    it = filename2frame.find(pos_file_stem);
    if (it != filename2frame.end())
    {
      ins_map[it->second] = kwiver::maptk::read_pos_file(fpath);
    }
  }
  // Warn if the POS file set is sparse compared to input frames
  if (!ins_map.empty())
  {
    // TODO: generated interpolated cameras for missing POS files.
    if (filename2frame.size() != ins_map.size())
    {
      std::cerr << "Warning: Input POS file-set is sparse compared to input "
                << "imagery! (not as many input POS files as there were input "
                << "images)"
                << std::endl;
    }

    kwiver::vital::simple_camera base_camera = base_camera_from_config(config->subblock("base_camera"));
    kwiver::vital::rotation_d ins_rot_offset = config->get_value<kwiver::vital::rotation_d>("ins:rotation_offset", kwiver::vital::rotation_d());
    input_cameras = kwiver::maptk::initialize_cameras_with_ins(ins_map, base_camera,
                                                               local_cs,
                                                               ins_rot_offset);
  }
  else
  {
    std::cerr << "ERROR: No POS files from input set match input image "
              << "frames. Check POS files!"
              << std::endl;
    return false;
  }

  return true;
}


// Load input KRTD cameras from file, matching against the given image
// filename map. Returns false if failure occurred.
bool
load_input_cameras_krtd(kwiver::vital::config_block_sptr config,
                        std::map<std::string, kwiver::vital::frame_id_t> const& filename2frame,
                        kwiver::maptk::local_geo_cs & local_cs,
                        kwiver::vital::camera_map::map_camera_t & input_cameras)
{
  kwiver::vital::scoped_cpu_timer t( "Initializing cameras from KRTD files" );

  // Collect files
  std::string krtd_files = config->get_value<std::string>("input_krtd_files");
  std::vector< kwiver::vital::path_t > files;
  if (!resolve_files(krtd_files, files))
  {
    std::cerr << "ERROR: Could not open KRTD file list." << std::endl;
    return false;
  }

  // Associating KRTD files to the frame ID of a matching input image based
  // on file stem naming.
  std::cerr << "loading KRTD input camera files" << std::endl;
  kwiver::vital::camera_map::map_camera_t krtd_cams;
  std::map<std::string, kwiver::vital::frame_id_t>::const_iterator it;
  VITAL_FOREACH(kwiver::vital::path_t const& fpath, files)
  {
    std::string krtd_file_stem = ST::GetFilenameWithoutLastExtension( fpath );
    it = filename2frame.find(krtd_file_stem);
    if (it != filename2frame.end())
    {
      kwiver::vital::camera_sptr cam = kwiver::vital::read_krtd_file(fpath);
      krtd_cams[it->second] = cam;
    }
  }

  // if krtd_map is empty, then there were no input krtd files that matched
  // input imagery.
  if (krtd_cams.empty())
  {
    std::cerr << "ERROR: No KRTD files from input set match input image "
              << "frames. Check KRTD input files!"
              << std::endl;
    return false;
  }
  else
  {
    // Warning if loaded KRTD camera set is sparse compared to input imagery
    // TODO: generated interpolated cameras for missing KRTD files.
    if (filename2frame.size() != krtd_cams.size())
    {
      std::cerr << "WARNING: Input KRTD camera set is sparse compared to input "
                << "imagery! (there wasn't a matching KRTD input file for "
                << "every input image file)"
                << std::endl;
    }
    input_cameras = krtd_cams;
  }
  return true;
}


// Generic configuration based input camera load function.
//
// The local_cs and input_cameras objects may or may not be updated based on
// configuration.
//
// Returns false if errors occurred, otherwise true. Returns true even if no
// input cameras loaded (check size of input_cameras structure).
bool load_input_cameras(kwiver::vital::config_block_sptr config,
                        std::map<std::string, kwiver::vital::frame_id_t> const& filename2frame,
                        kwiver::maptk::local_geo_cs & local_cs,
                        kwiver::vital::camera_map::map_camera_t & input_cameras)
{
  // configuration check assured mutual exclusivity
  if (config->get_value<std::string>("input_pos_files", "") != "")
  {
    return load_input_cameras_pos(config, filename2frame, local_cs, input_cameras);
  }
  else if (config->get_value<std::string>("input_krtd_files", "") != "")
  {
    return load_input_cameras_krtd(config, filename2frame, local_cs, input_cameras);
  }

  // No input specified
  return true;
}


#define print_config(config)                                            \
  do                                                                    \
  {                                                                     \
    VITAL_FOREACH( kwiver::vital::config_block_key_t key, config->available_values() ) \
    {                                                                   \
      std::cerr << "\t"                                                 \
           << key << " = " << config->get_value<kwiver::vital::config_block_key_t>(key) \
           << std::endl;                                                \
    }                                                                   \
  } while (false)


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
      << "Options:"
      << arg.GetHelp() << std::endl;
    return EXIT_SUCCESS;
  }

  // register the algorithm implementations
  std::string rel_plugin_path = kwiver::vital::get_executable_path() + "/../lib/maptk";
  kwiver::vital::algorithm_plugin_manager::instance().add_search_path(rel_plugin_path);
  kwiver::vital::algorithm_plugin_manager::instance().register_plugins();

  // Set config to algo chain
  // Get config from algo chain after set
  // Check config validity, store result
  //
  // If -o/--output-config given,
  //   output config result and notify of current (in)validity
  // Else error if provided config not valid.

  // Set up top level configuration w/ defaults where applicable.
  kwiver::vital::config_block_sptr config = kwiver::vital::config_block::empty_config();
  kwiver::vital::algo::bundle_adjust_sptr bundle_adjuster;
  kwiver::vital::algo::initialize_cameras_landmarks_sptr initializer;
  kwiver::vital::algo::triangulate_landmarks_sptr triangulator;
  kwiver::vital::algo::geo_map_sptr geo_mapper;
  kwiver::vital::algo::estimate_similarity_transform_sptr st_estimator;

  // If -c/--config given, read in confg file, merge in with default just generated
  if( ! opt_config.empty() )
  {
    const std::string prefix = kwiver::vital::get_executable_path() + "/..";
    config->merge_config(kwiver::vital::read_config_file(opt_config, "maptk",
                                                         MAPTK_VERSION, prefix));
  }

  //std::cerr << "[DEBUG] Config BEFORE set:" << std::endl;
  //print_config(config);

  kwiver::vital::algo::bundle_adjust::set_nested_algo_configuration("bundle_adjuster", config, bundle_adjuster);
  kwiver::vital::algo::triangulate_landmarks::set_nested_algo_configuration("triangulator", config, triangulator);
  kwiver::vital::algo::initialize_cameras_landmarks::set_nested_algo_configuration("initializer", config, initializer);
  kwiver::vital::algo::geo_map::set_nested_algo_configuration("geo_mapper", config, geo_mapper);
  kwiver::vital::algo::estimate_similarity_transform::set_nested_algo_configuration("st_estimator", config, st_estimator);

  //std::cerr << "[DEBUG] Config AFTER set:" << std::endl;
  //print_config(config);

  bool valid_config = check_config(config);

  if( ! opt_out_config.empty() )
  {
    kwiver::vital::config_block_sptr dflt_config = default_config();
    dflt_config->merge_config(config);
    config = dflt_config;
    kwiver::vital::algo::bundle_adjust::get_nested_algo_configuration("bundle_adjuster", config, bundle_adjuster);
    kwiver::vital::algo::triangulate_landmarks::get_nested_algo_configuration("triangulator", config, triangulator);
    kwiver::vital::algo::initialize_cameras_landmarks::get_nested_algo_configuration("initializer", config, initializer);
    kwiver::vital::algo::geo_map::get_nested_algo_configuration("geo_mapper", config, geo_mapper);
    kwiver::vital::algo::estimate_similarity_transform::get_nested_algo_configuration("st_estimator", config, st_estimator);

    //std::cerr << "[DEBUG] Given config output target: "
    //          << opt_out_config << std::endl;
    write_config_file(config, opt_out_config );
    if(valid_config)
    {
      std::cerr << "INFO: Configuration file contained valid parameters"
                << " and may be used for running" << std::endl;
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

  //
  // Read the track file
  //
  std::string track_file = config->get_value<std::string>("input_track_file");
  std::cerr << "loading track file: " << track_file <<std::endl;
  kwiver::vital::track_set_sptr tracks = kwiver::vital::read_track_file(track_file);

  std::cerr << "loaded "<<tracks->size()<<" tracks"<<std::endl;
  if( tracks->size() == 0 )
  {
    std::cerr << "No tracks loaded."
              << std::endl;
    return EXIT_FAILURE;
  }
  size_t min_track_len = config->get_value<size_t>("min_track_length");
  if( min_track_len > 1 )
  {
    kwiver::vital::scoped_cpu_timer t( "track filtering" );
    tracks = filter_tracks(tracks, min_track_len);
    std::cerr << "filtered down to "<<tracks->size()<<" long tracks"<<std::endl;

    // write out filtered tracks if output file is specified
    if (config->has_value("filtered_track_file"))
    {
      std::string out_track_file = config->get_value<std::string>("filtered_track_file");
      if( out_track_file != "" )
      {
        kwiver::vital::write_track_file(tracks, out_track_file);
      }
    }

    if( tracks->size() == 0 )
    {
      std::cerr << "All track have been filtered. "
                << "No tracks are longer than " << min_track_len << " frames. "
                << "Try decreasing \"min_track_len\""
                << std::endl;
      return EXIT_FAILURE;
    }
  }

  //
  // Read in image list file
  //
  // Also creating helper structures (i.e. frameID-to-filename and vise versa
  // maps).
  //
  std::string image_list_file = config->get_value<std::string>("image_list_file");
  std::ifstream image_list_ifs(image_list_file.c_str());
  if (!image_list_ifs)
  {
    std::cerr << "Error: Could not open image list file!" << std::endl;
    return EXIT_FAILURE;
  }
  std::vector<kwiver::vital::path_t> image_files;
  for (std::string line; std::getline(image_list_ifs, line); )
  {
    image_files.push_back(line);
  }
  // Since input tracks were generated over these frames, we can assume that
  // the frames are "in order" in that tracking followed this list in this given
  // order. As this is a single list, we assume that there are no gaps (same
  // assumptions as makde in tracking).
  // Creating forward and revese mappings for frame to file stem-name.
  std::vector<std::string> frame2filename;  // valid since we are assuming no frame gaps
  std::map<std::string, kwiver::vital::frame_id_t> filename2frame;
  VITAL_FOREACH(kwiver::vital::path_t i_file, image_files)
  {
    std::string i_file_stem = ST::GetFilenameWithoutLastExtension( i_file );
    filename2frame[i_file_stem] = static_cast<kwiver::vital::frame_id_t>(frame2filename.size());
    frame2filename.push_back(i_file_stem);
  }

  //
  // Create the local coordinate system
  //
  kwiver::maptk::local_geo_cs local_cs(geo_mapper);

  //
  // Initialize input and main cameras
  //

  // Initialize input camera map based on which input files were given, if any.
  // If input_cameras is empty after this method, then there were no input
  // camera files.
  //
  // Config check above ensures validity + mutual exclusivity of these options
  kwiver::vital::camera_map::map_camera_t input_cameras;
  if (!load_input_cameras(config, filename2frame, local_cs, input_cameras))
  {
    std::cerr << "ERROR: Failed to load input cameras" << std::endl;
    return EXIT_FAILURE;
  }

  // Copy input cameras into main camera map
  kwiver::vital::camera_map::map_camera_t cameras;
  kwiver::vital::landmark_map_sptr lm_map;
  kwiver::vital::camera_map_sptr input_cam_map(new kwiver::vital::simple_camera_map(input_cameras));
  if (input_cameras.size() != 0)
  {
    VITAL_FOREACH(kwiver::vital::camera_map::map_camera_t::value_type &v, input_cameras)
    {
      cameras[v.first] = v.second->clone();
    }
    // Triangulate initial landmarks based on cameras and tracks
    triangulator->triangulate(input_cam_map, tracks, lm_map);
  }
  kwiver::vital::camera_map_sptr cam_map;
  if(!cameras.empty())
  {
    cam_map = kwiver::vital::camera_map_sptr(new kwiver::vital::simple_camera_map(cameras));
  }

  kwiver::vital::landmark_map_sptr reference_landmarks(new kwiver::vital::simple_landmark_map());
  kwiver::vital::track_set_sptr reference_tracks(new kwiver::vital::simple_track_set());
  if (config->get_value<std::string>("input_reference_points_file", "") != "")
  {
    kwiver::vital::path_t ref_file = config->get_value<kwiver::vital::path_t>("input_reference_points_file");

    // Load up landmarks and assocaited tracks from file, (re)initializing
    // local coordinate system object to the reference.
    kwiver::maptk::load_reference_file(ref_file, local_cs, reference_landmarks, reference_tracks);
  }

  // apply necker reversal if requested
  bool necker_reverse_input = config->get_value<bool>("necker_reverse_input", false);
  if (necker_reverse_input)
  {
    std::cerr << "Applying Necker reversal" << std::endl;
    kwiver::maptk::necker_reverse(cam_map, lm_map);
  }

  bool init_unloaded_cams = config->get_value<bool>("initialize_unloaded_cameras", true);
  if (init_unloaded_cams)
  {
    if( cam_map )
    {
      cameras = cam_map->cameras();
    }
    VITAL_FOREACH(const kwiver::vital::frame_id_t& id, tracks->all_frame_ids())
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
      VITAL_FOREACH(const kwiver::vital::frame_id_t& id, tracks->all_frame_ids())
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
      VITAL_FOREACH(kwiver::vital::track_sptr const t, reference_tracks->tracks())
      {
        for (kwiver::vital::track::history_const_itr tsit = t->begin(); tsit != t->end(); ++tsit)
        {
          if (cams.count(tsit->frame_id) > 0)
          {
            sub_cams.insert(*cams.find(tsit->frame_id));
          }
        }
      }
      subsampled_cams = kwiver::vital::camera_map_sptr(new kwiver::vital::simple_camera_map(sub_cams));
    }

    cam_map = subsampled_cams;
    std::cerr << "subsampled down to "<<cam_map->size()<<" cameras"<<std::endl;
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

    double init_rmse = kwiver::maptk::reprojection_rmse(cam_map->cameras(),
                                                        lm_map->landmarks(),
                                                        tracks->tracks());
    std::cerr << "initial reprojection RMSE: " << init_rmse << std::endl;

    bundle_adjuster->optimize(cam_map, lm_map, tracks);

    double end_rmse = kwiver::maptk::reprojection_rmse(cam_map->cameras(),
                                                       lm_map->landmarks(),
                                                       tracks->tracks());
    std::cerr << "final reprojection RMSE: " << end_rmse << std::endl;
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
  if (st_estimator)
  {
    kwiver::vital::scoped_cpu_timer t_1( "--> st estimation and application" );
    std::cerr << "Estimating similarity transform from post-SBA to original space"
              << std::endl;

    // initialize identity transform
    kwiver::vital::similarity_d sim_transform;

    // Prioritize use of reference landmarks/tracks over use of POS files for
    // transformation out of SBA-space.
    if (reference_landmarks->size() > 0 && reference_tracks->size() > 0)
    {
      kwiver::vital::scoped_cpu_timer t_2( "similarity transform estimation from ref file" );
      std::cerr << "--> Using reference landmarks/tracks" << std::endl;

      // Generate corresponding landmarks in SBA-space based on transformed
      //    cameras and reference landmarks/tracks via triangulation.
      std::cerr << "--> Triangulating SBA-space reference landmarks from "
                << "reference tracks and post-SBA cameras" << std::endl;
      kwiver::vital::landmark_map_sptr sba_space_landmarks(new kwiver::vital::simple_landmark_map(reference_landmarks->landmarks()));
      triangulator->triangulate(cam_map, reference_tracks, sba_space_landmarks);

      double post_tri_rmse = kwiver::maptk::reprojection_rmse(cam_map->cameras(),
                                                              sba_space_landmarks->landmarks(),
                                                              reference_tracks->tracks());
      std::cerr << "--> Post-triangulation RMSE: " << post_tri_rmse << std::endl;

      // Estimate ST from sba-space to reference space.
      std::cerr << "--> Estimating transform to reference landmarks (from "
                << "SBA-space ref landmarks)" << std::endl;
      sim_transform = st_estimator->estimate_transform(sba_space_landmarks, reference_landmarks);
    }
    else if (input_cam_map->size() > 0)
    {
      kwiver::vital::scoped_cpu_timer t_2( "    similarity transform estimation from camera" );

      std::cerr << "--> Estimating transform to refined cameras "
                << "(from input cameras)" << std::endl;
      sim_transform = st_estimator->estimate_transform(cam_map, input_cam_map);
    }
    else
    {
      // In the absence of other information, use a canonical transformation
      sim_transform = kwiver::maptk::canonical_transform(cam_map, lm_map);
    }

    std::cerr << "--> Estimated Transformation: " << sim_transform
              << std::endl;

    // apply to cameras
    std::cerr << "--> Applying to cameras..." << std::endl;
    cam_map = kwiver::maptk::transform(cam_map, sim_transform);
    // apply to landmarks
    std::cerr << "--> Applying to landmarks..." << std::endl;
    lm_map = kwiver::maptk::transform(lm_map, sim_transform);
  }

  //
  // Compute landmark colors
  //
  lm_map = kwiver::maptk::compute_landmark_colors(*lm_map, *tracks);

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
    std::cerr << "Writing output POS files" << std::endl;
    kwiver::vital::scoped_cpu_timer t( "--> Writing output POS files" );

    kwiver::vital::path_t pos_dir = config->get_value<std::string>("output_pos_dir");
    // Create INS data from adjusted cameras for POS file output.
    typedef std::map<kwiver::vital::frame_id_t, kwiver::maptk::ins_data> ins_map_t;
    ins_map_t ins_map;
    update_ins_from_cameras(cam_map->cameras(), local_cs, ins_map);
    VITAL_FOREACH(const ins_map_t::value_type& p, ins_map)
    {
      kwiver::vital::path_t out_pos_file = pos_dir + "/" + frame2filename[p.first] + ".pos";
      write_pos_file( p.second, out_pos_file);
    }
    if (ins_map.size() == 0)
    {
      std::cerr << "--> INS map empty, no output POS files written" << std::endl;
    }
  }

  //
  // Write the output KRTD files
  //
  if( config->has_value("output_krtd_dir") )
  {
    std::cerr << "Writing output KRTD files" << std::endl;
    kwiver::vital::scoped_cpu_timer t("--> Writing output KRTD files" );

    kwiver::vital::path_t krtd_dir = config->get_value<std::string>("output_krtd_dir");
    typedef kwiver::vital::camera_map::map_camera_t::value_type cam_map_val_t;
    VITAL_FOREACH(cam_map_val_t const& p, cam_map->cameras())
    {
      kwiver::vital::path_t out_krtd_file = krtd_dir + "/" + frame2filename[p.first] + ".krtd";
      write_krtd_file( *p.second, out_krtd_file );
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
    std::cerr << "Exception caught: " << e.what() << std::endl;

    return EXIT_FAILURE;
  }
  catch (...)
  {
    std::cerr << "Unknown exception caught" << std::endl;

    return EXIT_FAILURE;
  }
}
