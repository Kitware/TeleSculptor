/*ckwg +29
 * Copyright 2014-2015 by Kitware, Inc.
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

#include <maptk/algo/bundle_adjust.h>
#include <maptk/algo/initialize_cameras_landmarks.h>
#include <maptk/algo/estimate_similarity_transform.h>
#include <maptk/algo/triangulate_landmarks.h>
#include <maptk/algo/geo_map.h>
#include <maptk/algorithm_plugin_manager.h>
#include <maptk/camera_io.h>
#include <maptk/config_block.h>
#include <maptk/config_block_io.h>
#include <maptk/eigen_io.h>
#include <maptk/exceptions.h>
#include <maptk/geo_reference_points_io.h>
#include <maptk/ins_data_io.h>
#include <maptk/landmark_map_io.h>
#include <maptk/local_geo_cs.h>
#include <maptk/metrics.h>
#include <maptk/track_set.h>
#include <maptk/track_set_io.h>
#include <maptk/transform.h>
#include <maptk/types.h>

#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/value_semantic.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/timer/timer.hpp>


namespace bfs = boost::filesystem;
namespace bpo = boost::program_options;


static maptk::config_block_sptr default_config()
{
  using namespace maptk;

  config_block_sptr config = config_block::empty_config("bundle_adjust_tracks_tool");

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

  algo::bundle_adjust::get_nested_algo_configuration("bundle_adjuster", config,
                                                     algo::bundle_adjust_sptr());
  algo::initialize_cameras_landmarks
      ::get_nested_algo_configuration("initializer", config,
                                      algo::initialize_cameras_landmarks_sptr());
  algo::triangulate_landmarks::get_nested_algo_configuration("triangulator", config,
                                                             algo::triangulate_landmarks_sptr());
  algo::geo_map::get_nested_algo_configuration("geo_mapper", config,
                                               algo::geo_map_sptr());
  algo::estimate_similarity_transform::get_nested_algo_configuration("st_estimator", config,
                                                                     algo::estimate_similarity_transform_sptr());

  return config;
}


static bool check_config(maptk::config_block_sptr config)
{
  bool config_valid = true;

#define MAPTK_CONFIG_FAIL(msg) \
  std::cerr << "Config Check Fail: " << msg << std::endl; \
  config_valid = false

  if (!config->has_value("input_track_file"))
  {
    MAPTK_CONFIG_FAIL("Not given a tracks file path");
  }
  else if (!bfs::exists(config->get_value<std::string>("input_track_file")))
  {
    MAPTK_CONFIG_FAIL("Given tracks file path doesn't point to an existing file.");
  }

  if (!config->has_value("image_list_file"))
  {
    MAPTK_CONFIG_FAIL("Not given an image list file");
  }
  else if (!bfs::exists(config->get_value<std::string>("image_list_file")))
  {
    MAPTK_CONFIG_FAIL("Given image list file path doesn't point to an existing file.");
  }

  // Checking input cameras and reference points file existance.
  bool input_pos = false,
       input_krtd = false;
  if (config->get_value<std::string>("input_pos_files", "") != "")
  {
    input_pos = true;
    if (!bfs::exists(config->get_value<std::string>("input_pos_files")))
    {
      MAPTK_CONFIG_FAIL("POS input path given, but doesn't point to an existing location.");
    }
  }
  if (config->get_value<std::string>("input_krtd_files", "") != "")
  {
    input_krtd = true;
    if (!bfs::exists(config->get_value<std::string>("input_krtd_files")))
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
    if (!bfs::exists(config->get_value<std::string>("input_reference_points_file")))
    {
      MAPTK_CONFIG_FAIL("Path given for input reference points file does not exist.");
    }
  }

  if (!maptk::algo::bundle_adjust::check_nested_algo_configuration("bundle_adjuster", config))
  {
    MAPTK_CONFIG_FAIL("Failed config check in bundle_adjuster algorithm.");
  }
  if (!maptk::algo::initialize_cameras_landmarks
            ::check_nested_algo_configuration("initializer", config))
  {
    MAPTK_CONFIG_FAIL("Failed config check in initializer algorithm.");
  }
  if (!maptk::algo::triangulate_landmarks::check_nested_algo_configuration("triangulator", config))
  {
    MAPTK_CONFIG_FAIL("Failed config check in triangulator algorithm.");
  }
  if (!maptk::algo::geo_map::check_nested_algo_configuration("geo_mapper", config))
  {
    MAPTK_CONFIG_FAIL("Failed config check in geo_mapper algorithm.");
  }
  if (config->has_value("st_estimator:type") && config->get_value<std::string>("st_estimator:type") != "")
  {
    if (!maptk::algo::estimate_similarity_transform::check_nested_algo_configuration("st_estimator", config))
    {
      MAPTK_CONFIG_FAIL("Failed config check in st_estimator algorithm.");
    }
  }

#undef MAPTK_CONFIG_FAIL

  return config_valid;
}


/// create a base camera instance from config options
maptk::camera_d
base_camera_from_config(maptk::config_block_sptr config)
{
  using namespace maptk;
  camera_intrinsics_d K(config->get_value<double>("focal_length"),
                        config->get_value<vector_2d>("principal_point"),
                        config->get_value<double>("aspect_ratio"),
                        config->get_value<double>("skew"));
  return camera_d(vector_3d(0,0,-1), rotation_d(), K);
}


/// filter track set by removing short tracks
maptk::track_set_sptr
filter_tracks(maptk::track_set_sptr tracks, size_t min_length)
{
  using namespace maptk;
  std::vector<track_sptr> trks = tracks->tracks();
  std::vector<track_sptr> good_trks;
  BOOST_FOREACH(track_sptr t, trks)
  {
    if( t->size() >= min_length )
    {
      good_trks.push_back(t);
    }
  }
  return track_set_sptr(new simple_track_set(good_trks));
}


/// Subsample a every Nth camera, where N is specfied by factor
/**
 * Uses camera frame numbers to determine subsample. This is fine when we
 * assume the cameras given are sequential and always start with frame 0.
 * This will behave in possibly undesired ways when the given cameras are not in
 * sequential frame order, or the first camera's frame is not a multiple of
 * \c factor.
 */
maptk::camera_map_sptr
subsample_cameras(maptk::camera_map_sptr cameras, unsigned factor)
{
  using namespace maptk;
  camera_map::map_camera_t cams = cameras->cameras();
  camera_map::map_camera_t sub_cams;
  BOOST_FOREACH(const camera_map::map_camera_t::value_type& p, cams)
  {
    if(p.first % factor == 0)
    {
      sub_cams.insert(p);
    }
  }
  return camera_map_sptr(new simple_camera_map(sub_cams));
}


// return a sorted list of files in a directory
std::vector<bfs::path>
files_in_dir(const bfs::path& dir)
{
  bfs::directory_iterator it(dir), eod;
  std::vector<bfs::path> files;
  BOOST_FOREACH(bfs::path const &p, std::make_pair(it, eod))
  {
    files.push_back(p);
  }
  std::sort(files.begin(), files.end());
  return files;
}


/// Return a list of file paths either from a directory of files or from a
/// list of file paths
///
/// Returns false if we were given a file list and the file could not be
/// opened. Otherwise returns true.
bool
resolve_files(maptk::path_t const &p, std::vector<maptk::path_t> &files)
{
  if (bfs::is_directory(p))
  {
    files = files_in_dir(p);
  }
  else
  {
    std::ifstream ifs(p.string().c_str());
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
load_input_cameras_pos(maptk::config_block_sptr config,
                       std::map<std::string, maptk::frame_id_t> const& filename2frame,
                       maptk::local_geo_cs & local_cs,
                       maptk::camera_map::map_camera_t & input_cameras)
{
  boost::timer::auto_cpu_timer t("Initializing cameras from POS files: %t sec CPU, %w sec wall\n");

  std::string pos_files = config->get_value<std::string>("input_pos_files");
  std::vector<bfs::path> files;
  if (!resolve_files(pos_files, files))
  {
    std::cerr << "ERROR: Could not open POS file list." << std::endl;
    return false;
  }

  std::cerr << "loading POS files" <<std::endl;
  // Associating POS file to frame ID based on whether its filename stem is
  // the same as an image in the given image list (map created above).
  std::map<maptk::frame_id_t, maptk::ins_data> ins_map;
  std::map<std::string, maptk::frame_id_t>::const_iterator it;
  BOOST_FOREACH(maptk::path_t const& fpath, files)
  {
    std::string pos_file_stem = fpath.stem().string();
    it = filename2frame.find(pos_file_stem);
    if (it != filename2frame.end())
    {
      ins_map[it->second] = maptk::read_pos_file(fpath);
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

    maptk::camera_d base_camera = base_camera_from_config(config->subblock("base_camera"));
    maptk::rotation_d ins_rot_offset = config->get_value<maptk::rotation_d>("ins:rotation_offset", maptk::rotation_d());
    input_cameras = maptk::initialize_cameras_with_ins(ins_map, base_camera,
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
load_input_cameras_krtd(maptk::config_block_sptr config,
                        std::map<std::string, maptk::frame_id_t> const& filename2frame,
                        maptk::local_geo_cs & local_cs,
                        maptk::camera_map::map_camera_t & input_cameras)
{
  boost::timer::auto_cpu_timer t("Initializing cameras from KRTD files: %t sec CPU, %w sec wall\n");

  // Collect files
  std::string krtd_files = config->get_value<std::string>("input_krtd_files");
  std::vector<bfs::path> files;
  if (!resolve_files(krtd_files, files))
  {
    std::cerr << "ERROR: Could not open KRTD file list." << std::endl;
    return false;
  }

  // Associating KRTD files to the frame ID of a matching input image based
  // on file stem naming.
  std::cerr << "loading KRTD input camera files" << std::endl;
  maptk::camera_map::map_camera_t krtd_cams;
  std::map<std::string, maptk::frame_id_t>::const_iterator it;
  BOOST_FOREACH(maptk::path_t const& fpath, files)
  {
    std::string krtd_file_stem = fpath.stem().string();
    it = filename2frame.find(krtd_file_stem);
    if (it != filename2frame.end())
    {
      maptk::camera_sptr cam(new maptk::camera_d(maptk::read_krtd_file(fpath)));
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
bool load_input_cameras(maptk::config_block_sptr config,
                        std::map<std::string, maptk::frame_id_t> const& filename2frame,
                        maptk::local_geo_cs & local_cs,
                        maptk::camera_map::map_camera_t & input_cameras)
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


#define print_config(config) \
  do \
  { \
    BOOST_FOREACH( maptk::config_block_key_t key, config->available_values() ) \
    { \
      std::cerr << "\t" \
           << key << " = " << config->get_value<maptk::config_block_key_t>(key) \
           << std::endl; \
    } \
  } while (false)


static int maptk_main(int argc, char const* argv[])
{
  // register the algorithm implementations
  maptk::algorithm_plugin_manager::instance().register_plugins();

  // define/parse CLI options
  bpo::options_description opt_desc;
  opt_desc.add_options()
    ("help,h", "output help message and exit")
    ("config,c",
     bpo::value<maptk::path_t>(),
     "Configuration file for the tool.")
    ("output-config,o",
     bpo::value<maptk::path_t>(),
     "Output a configuration.This may be seeded with"
     " a configuration file from -c/--config.")
    ;
  bpo::variables_map vm;

  try
  {
    bpo::store(bpo::parse_command_line(argc, argv, opt_desc), vm);
  }
  catch (bpo::unknown_option const& e)
  {
    std::cerr << "Error: unknown option " << e.get_option_name() << std::endl;
    return EXIT_FAILURE;
  }

  bpo::notify(vm);

  if(vm.count("help"))
  {
    std::cerr << opt_desc << std::endl;
    return EXIT_SUCCESS;
  }

  // Set config to algo chain
  // Get config from algo chain after set
  // Check config validity, store result
  //
  // If -o/--output-config given,
  //   output config result and notify of current (in)validity
  // Else error if provided config not valid.

  namespace algo = maptk::algo;

  // Set up top level configuration w/ defaults where applicable.
  maptk::config_block_sptr config = maptk::config_block::empty_config();
  algo::bundle_adjust_sptr bundle_adjuster;
  algo::initialize_cameras_landmarks_sptr initializer;
  algo::triangulate_landmarks_sptr triangulator;
  algo::geo_map_sptr geo_mapper;
  algo::estimate_similarity_transform_sptr st_estimator;

  // If -c/--config given, read in confg file, merge in with default just generated
  if(vm.count("config"))
  {
    //std::cerr << "[DEBUG] Given config file: " << vm["config"].as<maptk::path_t>() << std::endl;
    config->merge_config(maptk::read_config_file(vm["config"].as<maptk::path_t>()));
  }

  //std::cerr << "[DEBUG] Config BEFORE set:" << std::endl;
  //print_config(config);

  algo::bundle_adjust::set_nested_algo_configuration("bundle_adjuster", config, bundle_adjuster);
  algo::triangulate_landmarks::set_nested_algo_configuration("triangulator", config, triangulator);
  algo::initialize_cameras_landmarks::set_nested_algo_configuration("initializer", config, initializer);
  algo::geo_map::set_nested_algo_configuration("geo_mapper", config, geo_mapper);
  algo::estimate_similarity_transform::set_nested_algo_configuration("st_estimator", config, st_estimator);

  //std::cerr << "[DEBUG] Config AFTER set:" << std::endl;
  //print_config(config);

  bool valid_config = check_config(config);

  if(vm.count("output-config"))
  {
    maptk::config_block_sptr dflt_config = default_config();
    dflt_config->merge_config(config);
    config = dflt_config;
    algo::bundle_adjust::get_nested_algo_configuration("bundle_adjuster", config, bundle_adjuster);
    algo::triangulate_landmarks::get_nested_algo_configuration("triangulator", config, triangulator);
    algo::initialize_cameras_landmarks::get_nested_algo_configuration("initializer", config, initializer);
    algo::geo_map::get_nested_algo_configuration("geo_mapper", config, geo_mapper);
    algo::estimate_similarity_transform::get_nested_algo_configuration("st_estimator", config, st_estimator);

    //std::cerr << "[DEBUG] Given config output target: "
    //          << vm["output-config"].as<maptk::path_t>() << std::endl;
    write_config_file(config, vm["output-config"].as<maptk::path_t>());
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
  maptk::track_set_sptr tracks = maptk::read_track_file(track_file);

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
    boost::timer::auto_cpu_timer t("track filtering: %t sec CPU, %w sec wall\n");
    tracks = filter_tracks(tracks, min_track_len);
    std::cerr << "filtered down to "<<tracks->size()<<" long tracks"<<std::endl;

    // write out filtered tracks if output file is specified
    if (config->has_value("filtered_track_file"))
    {
      std::string out_track_file = config->get_value<std::string>("filtered_track_file");
      if( out_track_file != "" )
      {
        maptk::write_track_file(tracks, out_track_file);
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
  std::vector<maptk::path_t> image_files;
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
  std::map<std::string, maptk::frame_id_t> filename2frame;
  BOOST_FOREACH(maptk::path_t i_file, image_files)
  {
    std::string i_file_stem = i_file.stem().string();
    filename2frame[i_file_stem] = static_cast<maptk::frame_id_t>(frame2filename.size());
    frame2filename.push_back(i_file_stem);
  }

  //
  // Create the local coordinate system
  //
  maptk::local_geo_cs local_cs(geo_mapper);

  //
  // Initialize input and main cameras
  //

  // Initialize input camera map based on which input files were given, if any.
  // If input_cameras is empty after this method, then there were no input
  // camera files.
  //
  // Config check above ensures validity + mutual exclusivity of these options
  maptk::camera_map::map_camera_t input_cameras;
  if (!load_input_cameras(config, filename2frame, local_cs, input_cameras))
  {
    std::cerr << "ERROR: Failed to load input cameras" << std::endl;
    return EXIT_FAILURE;
  }

  // Copy input cameras into main camera map
  //
  // If there were no input cameras, initialize fixed location cameras to main
  // camera map.
  maptk::camera_map::map_camera_t cameras;
  maptk::landmark_map_sptr lm_map;
  maptk::camera_map_sptr input_cam_map(new maptk::simple_camera_map(input_cameras));
  if (input_cameras.size() != 0)
  {
    BOOST_FOREACH(maptk::camera_map::map_camera_t::value_type &v, input_cameras)
    {
      cameras[v.first] = v.second->clone();
    }
    // Triangulate initial landmarks based on cameras and tracks
    triangulator->triangulate(input_cam_map, tracks, lm_map);
  }

  //
  // Initialize cameras and landmarks
  //
  maptk::camera_map_sptr cam_map;
  if(!cameras.empty())
  {
    cam_map = maptk::camera_map_sptr(new maptk::simple_camera_map(cameras));
  }
  {
    boost::timer::auto_cpu_timer t("Initializing cameras and landmarks: %t sec CPU, %w sec wall\n");
    initializer->initialize(cam_map, lm_map, tracks);
  }

  maptk::landmark_map_sptr reference_landmarks(new maptk::simple_landmark_map());
  maptk::track_set_sptr reference_tracks(new maptk::simple_track_set());
  if (config->get_value<std::string>("input_reference_points_file", "") != "")
  {
    maptk::path_t ref_file = config->get_value<maptk::path_t>("input_reference_points_file");

    // Load up landmarks and assocaited tracks from file, (re)initializing
    // local coordinate system object to the reference.
    maptk::load_reference_file(ref_file, local_cs, reference_landmarks, reference_tracks);
  }

  //
  // Cut down input cameras if a sub-sample rate was specified
  //
  unsigned int cam_samp_rate = config->get_value<unsigned int>("camera_sample_rate");
  if(cam_samp_rate > 1)
  {
    boost::timer::auto_cpu_timer t("Tool-level sub-sampling: %t sec CPU, %w sec wall\n");

    maptk::camera_map_sptr subsampled_cams = subsample_cameras(cam_map, cam_samp_rate);

    // If we were given reference landmarks and tracks, make sure to include
    // the cameras for frames reference track states land on. Required for
    // sba-space landmark triangulation and correlation later.
    if (reference_tracks->size() > 0)
    {
      maptk::camera_map::map_camera_t cams = cam_map->cameras(),
                                      sub_cams = subsampled_cams->cameras();
      // for each track state in each reference track, make sure that the
      // state's frame's camera is in the sub-sampled set of cameras
      BOOST_FOREACH(maptk::track_sptr const t, reference_tracks->tracks())
      {
        for (maptk::track::history_const_itr tsit = t->begin(); tsit != t->end(); ++tsit)
        {
          if (cams.count(tsit->frame_id) > 0)
          {
            sub_cams.insert(*cams.find(tsit->frame_id));
          }
        }
      }
      subsampled_cams = maptk::camera_map_sptr(new maptk::simple_camera_map(sub_cams));
    }

    cam_map = subsampled_cams;
    std::cerr << "subsampled down to "<<cam_map->size()<<" cameras"<<std::endl;
  }

  //
  // Run bundle adjustment
  //
  { // scope block
    boost::timer::auto_cpu_timer t("Tool-level SBA algorithm: %t sec CPU, %w sec wall\n");

    double init_rmse = maptk::reprojection_rmse(cam_map->cameras(),
                                                lm_map->landmarks(),
                                                tracks->tracks());
    std::cerr << "initial reprojection RMSE: " << init_rmse << std::endl;

    bundle_adjuster->optimize(cam_map, lm_map, tracks);

    double end_rmse = maptk::reprojection_rmse(cam_map->cameras(),
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
    boost::timer::auto_cpu_timer t_1("--> st estimation and application: %t sec "
                                     "CPU, %w sec wall\n");
    std::cerr << "Estimating similarity transform from post-SBA to original space"
              << std::endl;

    // initialize identity transform
    maptk::similarity_d sim_transform;

    // Prioritize use of reference landmarks/tracks over use of POS files for
    // transformation out of SBA-space.
    if (reference_landmarks->size() > 0 && reference_tracks->size() > 0)
    {
      boost::timer::auto_cpu_timer t_2("similarity transform estimation from "
                                       "ref file: %t sec CPU, %w sec wall\n");
      std::cerr << "--> Using reference landmarks/tracks" << std::endl;

      // Generate corresponding landmarks in SBA-space based on transformed
      //    cameras and reference landmarks/tracks via triangulation.
      std::cerr << "--> Triangulating SBA-space reference landmarks from "
                << "reference tracks and post-SBA cameras" << std::endl;
      maptk::landmark_map_sptr sba_space_landmarks(new maptk::simple_landmark_map(reference_landmarks->landmarks()));
      triangulator->triangulate(cam_map, reference_tracks, sba_space_landmarks);

      double post_tri_rmse = maptk::reprojection_rmse(cam_map->cameras(),
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
      boost::timer::auto_cpu_timer t_2("    %t sec CPU, %w sec wall\n");

      std::cerr << "--> Estimating transform to refined cameras "
                << "(from input cameras)" << std::endl;
      sim_transform = st_estimator->estimate_transform(cam_map, input_cam_map);
    }
    else
    {
      // In the absence of other information, use a canonical transformation
      sim_transform = canonical_transform(cam_map, lm_map);
    }

    std::cerr << "--> Estimated Transformation: " << sim_transform
              << std::endl;

    // apply to cameras
    std::cerr << "--> Applying to cameras..." << std::endl;
    cam_map = maptk::transform(cam_map, sim_transform);
    // apply to landmarks
    std::cerr << "--> Applying to landmarks..." << std::endl;
    lm_map = maptk::transform(lm_map, sim_transform);
  }

  //
  // Write the output PLY file
  //
  if( config->has_value("output_ply_file") )
  {
    boost::timer::auto_cpu_timer t("writing output PLY file: %t sec CPU, %w sec wall\n");
    std::string ply_file = config->get_value<std::string>("output_ply_file");
    write_ply_file(lm_map, ply_file);
  }

  //
  // Write the output POS files
  //
  if( config->has_value("output_pos_dir") )
  {
    std::cerr << "Writing output POS files" << std::endl;
    boost::timer::auto_cpu_timer t("--> %t sec CPU, %w sec wall\n");

    bfs::path pos_dir = config->get_value<std::string>("output_pos_dir");
    // Create INS data from adjusted cameras for POS file output.
    typedef std::map<maptk::frame_id_t, maptk::ins_data> ins_map_t;
    ins_map_t ins_map;
    maptk::update_ins_from_cameras(cam_map->cameras(), local_cs, ins_map);
    BOOST_FOREACH(const ins_map_t::value_type& p, ins_map)
    {
      bfs::path out_pos_file = pos_dir / (frame2filename[p.first] + ".pos");
      write_pos_file(p.second, out_pos_file);
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
    boost::timer::auto_cpu_timer t("--> %t sec CPU, %w sec wall\n");

    bfs::path krtd_dir = config->get_value<std::string>("output_krtd_dir");
    typedef maptk::camera_map::map_camera_t::value_type cam_map_val_t;
    BOOST_FOREACH(const cam_map_val_t& p, cam_map->cameras())
    {
      bfs::path out_krtd_file = krtd_dir / (frame2filename[p.first] + ".krtd");
      write_krtd_file(*p.second, out_krtd_file);
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
