/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
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

#include <maptk/modules.h>

#include <maptk/core/algo/bundle_adjust.h>
#include <maptk/core/algo/estimate_similarity_transform.h>
#include <maptk/core/algo/triangulate_landmarks.h>
#include <maptk/core/algo/geo_map.h>
#include <maptk/core/camera_io.h>
#include <maptk/core/config_block.h>
#include <maptk/core/config_block_io.h>
#include <maptk/core/exceptions.h>
#include <maptk/core/ins_data_io.h>
#include <maptk/core/landmark_map_io.h>
#include <maptk/core/local_geo_cs.h>
#include <maptk/core/metrics.h>
#include <maptk/core/track_set.h>
#include <maptk/core/track_set_io.h>
#include <maptk/core/transform.h>
#include <maptk/core/types.h>

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

  config->set_value("image_list_file", "",
                    "Path to the input image list file used to generated the "
                    "input tracks.");

  config->set_value("input_pos_files", "",
                    "A directory containing the input POS files, or a text file"
                    "containing a newline-separated list of POS files. "
                    "This is optional, leave blank to ignore. This is "
                    "mutually exclusive with the input_reference_points "
                    "option when using an st_estimator.");

  config->set_value("input_reference_points_file", "",
                    "File containing reference points to use for reprojection "
                    "or results into the original coordinate system. This "
                    "option is mutually exclusive with input_pos_files "
                    "when using an st_estimator. When not using an "
                    "st_estimator this option is ignored.");

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

  bool has_input_pos_files=false,
       has_input_ref_pt_list=false;
  if (config->get_value<std::string>("input_pos_files", "") != "")
  {
    if(!bfs::exists(config->get_value<std::string>("input_pos_files")))
    {
      MAPTK_CONFIG_FAIL("POS input path given, but doesn't point to an existing location.");
    }
    else // valid pos file list or path
      has_input_pos_files = true;
  }
  if (config->get_value<std::string>("input_reference_points_file", "") != "")
  {
    if (!bfs::exists(config->get_value<std::string>("input_reference_points_file")))
    {
      MAPTK_CONFIG_FAIL("Path given for input reference points file does not exist.");
    }
    else // valid ref pts file
      has_input_ref_pt_list = true;
  }

  if (!maptk::algo::bundle_adjust::check_nested_algo_configuration("bundle_adjuster", config))
  {
    MAPTK_CONFIG_FAIL("Failed config check in bundle_adjuster algorithm.");
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

  if (!(   !config->has_value("st_estimator:type")
        || (config->get_value<std::string>("st_estimator:type") == "")
        || maptk::algo::estimate_similarity_transform::check_nested_algo_configuration("st_estimator", config)
       ))
  {
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


/// Load landmarks and tracks from reference points file.
/**
 * Initializes and uses a local_geo_cs object given to transform reference
 * landmarks into a local coordinate system. The newly initialize lgcs is
 * passed back up by reference. Previous initialization of the given lgcs
 * is overwritten.
 */
void
load_reference_file(maptk::path_t const& reference_file,
                    maptk::local_geo_cs & lgcs,
                    maptk::landmark_map_sptr & ref_landmarks,
                    maptk::track_set_sptr & ref_track_set)
{
  using namespace maptk;
  using namespace std;

  // Read in file, creating a landmark map and a vector of tracks, associated
  // via IDs
  std::ifstream input_stream(reference_file.c_str(), std::fstream::in);
  if (!input_stream)
  {
    throw file_not_found_exception(reference_file, "Could not open reference points file!");
  }

  // pre-allocated vars for loop
  landmark_id_t cur_id = 1;
  frame_id_t frm;
  vector_2d feat_loc;
  vector_3d vec(0,0,0);
  double x, y;
  int zone;
  bool northp;
  // used to stream file lines into data types
  std::istringstream ss;

  landmark_map::map_landmark_t reference_lms;
  std::vector<track_sptr> reference_tracks;

  // Resetting lgcs' logical initialization
  lgcs.set_utm_origin(vec);
  lgcs.set_utm_origin_zone(-1);
  // Mean position of all landmarks.
  vector_3d mean(0,0,0);

  // TODO: put in try-catch around >>'s in case we have an ill-formatted file,
  // or there's a parse error
  cerr << "[load_reference_file] Reading from file: " << reference_file << endl;
  for (std::string line; std::getline(input_stream, line);)
  {
    ss.clear();
    ss.str(line);

    // input landmarks are given in lon/lat/alt format (ignoring alt for now)
    ss >> vec;

    // When this is called the first time, setzone is given a -1, which is the
    // default for the function.
    lgcs.geo_map_algo()->latlon_to_utm(vec.y(), vec.x(), x, y, zone, northp,
                                       lgcs.utm_origin_zone());
    vec[0] = x; vec[1] = y; vec[2] = 0;
    mean += vec;

    // Use the zone of the first input landmark as the base zone from which we
    // interpret all other geo-positions with respect to.
    if (lgcs.utm_origin_zone() == -1)
    {
      cerr << "[load_reference_file] lgcs zone: " << zone << endl;
      lgcs.set_utm_origin_zone(zone);
    }

    cerr << "[load_reference_file] landmark " << cur_id << " position :: " << std::setprecision(12) << vec << endl;
    reference_lms[cur_id] = landmark_sptr(new landmark_d(vec));

    // while there's still input left, read in track states
    cerr << "[] track:" << endl;
    track_sptr lm_track(new track());
    lm_track->set_id(static_cast<track_id_t>(cur_id));
    while (ss.peek() != std::char_traits<char>::eof())
    {
      ss >> frm;
      ss >> feat_loc;
      lm_track->append(track::track_state(frm, feature_sptr(new feature_d(feat_loc)), descriptor_sptr()));
      cerr << "[]\t- " << frm << " :: " << feat_loc << endl;
    }
    reference_tracks.push_back(lm_track);

    ++cur_id;
  }

  // Initialize lgcs center
  mean /= reference_lms.size();
  lgcs.set_utm_origin(mean);
  cerr << "[load_reference_file] mean position: " << mean << endl;

  // Scan through reference landmarks, adjusting their location by the lgcs
  // origin.
  cerr << "[load_reference_file] transforming lm locations..." << endl;
  BOOST_FOREACH(landmark_map::map_landmark_t::value_type & p, reference_lms)
  {
    dynamic_cast<landmark_d*>(p.second.get())->set_loc(p.second->loc() - mean);
    cerr << "[load_reference_file] -- " << p.first << " :: " << p.second->loc() << endl;
  }

  ref_landmarks = landmark_map_sptr(new simple_landmark_map(reference_lms));
  ref_track_set = track_set_sptr(new simple_track_set(reference_tracks));
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
  // register the algorithms in the various modules for dynamic look-up
  maptk::register_modules();

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
  std::cout << "loading track file: " << track_file <<std::endl;
  maptk::track_set_sptr tracks = maptk::read_track_file(track_file);

  std::cout << "loaded "<<tracks->size()<<" tracks"<<std::endl;
  size_t min_track_len = config->get_value<size_t>("min_track_length");
  if( min_track_len > 1 )
  {
    boost::timer::auto_cpu_timer t("track filtering: %t sec CPU, %w sec wall\n");
    tracks = filter_tracks(tracks, min_track_len);
    std::cout << "filtered down to "<<tracks->size()<<" long tracks"<<std::endl;
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
  maptk::camera_d base_camera = base_camera_from_config(config->subblock("base_camera"));

  //
  // Initialize all landmarks to the origin
  //
  std::set<maptk::track_id_t> track_ids = tracks->all_track_ids();
  maptk::landmark_map::map_landmark_t landmarks;
  BOOST_FOREACH(const maptk::track_id_t& tid, track_ids)
  {
    maptk::landmark_sptr lm(new maptk::landmark_d(maptk::vector_3d(0,0,0)));
    landmarks[static_cast<maptk::landmark_id_t>(tid)] = lm;
  }
  maptk::landmark_map_sptr lm_map(new maptk::simple_landmark_map(landmarks));

  //
  // Initialize cameras
  //
  typedef std::map<maptk::frame_id_t, maptk::ins_data> ins_map_t;
  ins_map_t ins_map;
  maptk::camera_map::map_camera_t cameras;

  // Vars that are used/populated when POS files are given.
  maptk::camera_map::map_camera_t pos_cameras;
  maptk::rotation_d ins_rot_offset = config->get_value<maptk::rotation_d>("ins:rotation_offset",
                                                                          maptk::rotation_d());
  // Vars that are used/populated if reference points file given.
  maptk::landmark_map_sptr reference_landmarks(new maptk::simple_landmark_map());
  maptk::track_set_sptr reference_tracks(new maptk::simple_track_set());

  // if POS files are available, use them to initialize the cameras
  if( config->get_value<std::string>("input_pos_files", "") != "" )
  {
    boost::timer::auto_cpu_timer t("Initializing cameras from POS files: %t sec CPU, %w sec wall\n");

    std::string pos_files = config->get_value<std::string>("input_pos_files");
    std::vector<bfs::path> files;
    if( bfs::is_directory(pos_files) )
    {
      files = files_in_dir(pos_files);
    }
    else
    {
      std::ifstream ifs(pos_files.c_str());
      if (!ifs)
      {
        std::cerr << "Error: Could not open POS file list "
                  << "\"" <<pos_files << "\""
                  << std::endl;
        return EXIT_FAILURE;
      }
      for (std::string line; std::getline(ifs,line); )
      {
        files.push_back(line);
      }
    }

    std::cout << "loading POS files" <<std::endl;
    // Associating POS file to frame ID based on whether its filename stem is
    // the same as an image in the given image list (map created above).
    BOOST_FOREACH(maptk::path_t const& fpath, files)
    {
      std::string pos_file_stem = fpath.stem().string();
      if (filename2frame.count(pos_file_stem))
      {
        ins_map[filename2frame[pos_file_stem]] = maptk::read_pos_file(fpath);
      }
    }
    // Warn if the POS file set is sparse compared to input frames
    if (!ins_map.empty())
    {
      // TODO: generated interpolated cameras for missing POS files.
      //       -> Q: Should this still a thing with the introduction of HSBA?
      if (filename2frame.size() != ins_map.size())
      {
        std::cerr << "Warning: Input POS file-set is sparse compared to input "
                  << "imagery! (not as many input POS files as there were input "
                  << "images)"
                  << std::endl;
      }

      cameras = maptk::initialize_cameras_with_ins(ins_map, base_camera, local_cs,
                                                   ins_rot_offset);
      // Creating duplicate cameras structure signifying POS files were input
      BOOST_FOREACH(maptk::camera_map::map_camera_t::value_type &v, cameras)
      {
        pos_cameras[v.first] = v.second->clone();
      }

      maptk::camera_map_sptr cam_map(new maptk::simple_camera_map(cameras));
      // triangulate to provide initial point locations
      triangulator->triangulate(cam_map, tracks, lm_map);
    }
    else
    {
      std::cerr << "ERROR: No POS files from input set match input image "
                << "frames. Check POS files!"
                << std::endl;
      return EXIT_FAILURE;
    }
  }
  // if no POS files, then initialize all cameras to a fixed location
  else
  {
    std::set<maptk::frame_id_t> frames = tracks->all_frame_ids();
    BOOST_FOREACH(const maptk::frame_id_t& fid, frames)
    {
      cameras[fid] = maptk::camera_sptr(new maptk::camera_d(base_camera));
    }
  }

  if (config->get_value<std::string>("input_reference_points_file", "") != "")
  {
    maptk::path_t ref_file = config->get_value<maptk::path_t>("input_reference_points_file");

    // Load up landmarks and assocaited tracks from file, (re)initializing local coordinate system object
    load_reference_file(ref_file, local_cs, reference_landmarks, reference_tracks);
  }

  maptk::camera_map_sptr cam_map(new maptk::simple_camera_map(cameras)),
                         orig_cam_map(new maptk::simple_camera_map(pos_cameras));

  std::cout << "initialized "<<cam_map->size()<<" cameras"<<std::endl;
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
    std::cout << "subsampled down to "<<cam_map->size()<<" cameras"<<std::endl;
  }

  //
  // Run bundle adjustment
  //
  { // scope block
    boost::timer::auto_cpu_timer t("Tool-level SBA algorithm: %t sec CPU, %w sec wall\n");

    double init_rmse = maptk::reprojection_rmse(cam_map->cameras(),
                                                lm_map->landmarks(),
                                                tracks->tracks());
    std::cout << "initial reprojection RMSE: " << init_rmse << std::endl;

    bundle_adjuster->optimize(cam_map, lm_map, tracks);

    double end_rmse = maptk::reprojection_rmse(cam_map->cameras(),
                                               lm_map->landmarks(),
                                               tracks->tracks());
    std::cout << "final reprojection RMSE: " << end_rmse << std::endl;
  }


  //
  // Adjust cameras/landmarks based on input cameras/reference points
  //
  // If we were given POS files / reference points as input, compute a
  // similarity transform from the refined cameras to the POS file / reference
  // point structures. Then, apply the estimated transform to the refined
  // camera positions and landmarks.
  //
  if (st_estimator)
  {
    boost::timer::auto_cpu_timer t_1("st estimation and application: %t sec "
                                     "CPU, %w sec wall\n");

    // initialize identity transform
    maptk::similarity_d sim_transform;

    // Prioritize use of reference landmarks/tracks over use of POS files for
    // transformation out of SBA-space.
    if (reference_landmarks->size() > 0 && reference_tracks->size() > 0)
    {
      boost::timer::auto_cpu_timer t_2("similarity transform estimation from "
                                       "ref file: %t sec CPU, %w sec wall\n");

      // Generate corresponding landmarks in SBA-space based on transformed
      //    cameras and reference landmarks/tracks via triangulation.
      maptk::landmark_map_sptr sba_space_landmarks(new maptk::simple_landmark_map(reference_landmarks->landmarks()));
      triangulator->triangulate(cam_map, reference_tracks, sba_space_landmarks);

      double post_tri_rmse = maptk::reprojection_rmse(cam_map->cameras(),
                                                      sba_space_landmarks->landmarks(),
                                                      reference_tracks->tracks());
      std::cerr << "Post reference triangulation RMSE: " << post_tri_rmse << std::endl;

      // Estimate ST from sba-space to reference space.
      sim_transform = st_estimator->estimate_transform(sba_space_landmarks, reference_landmarks);
    }
    else if (pos_cameras.size() > 0)
    {
      boost::timer::auto_cpu_timer t_2("similarity transform estimation from "
                                       "POS cams: %t sec CPU, %w sec wall\n");

      std::cout << "Estimating and applying similarity transform to refined "
                << "cameras (from POS files)" << std::endl;
      sim_transform = st_estimator->estimate_transform(cam_map, orig_cam_map);

    }

    std::cerr << "--> Estimated Transformation:" << std::endl
              << sim_transform << std::endl;

    // apply to cameras
    std::cout << "--> Applying to cameras..." << std::endl;
    cam_map = maptk::transform(cam_map, sim_transform);
    // apply to landmarks
    std::cout << "--> Applying to landmarks..." << std::endl;
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
    boost::timer::auto_cpu_timer t("writing output POS file(s): %t sec CPU, %w sec wall\n");
    bfs::path pos_dir = config->get_value<std::string>("output_pos_dir");
    // update ins_map with refined data. Its ok if ins map is empty.
    maptk::update_ins_from_cameras(cam_map->cameras(), local_cs, ins_map);
    BOOST_FOREACH(const ins_map_t::value_type& p, ins_map)
    {
      bfs::path out_pos_file = pos_dir / (frame2filename[p.first] + ".pos");
      write_pos_file(p.second, out_pos_file);
    }
  }

  //
  // Write the output KRTD files
  //
  if( config->has_value("output_krtd_dir") )
  {
    boost::timer::auto_cpu_timer t("writing output KRTD file(s): %t sec CPU, %w sec wall\n");
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
