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
#include <fstream>
#include <sstream>
#include <exception>
#include <string>
#include <vector>

#include <maptk/modules.h>

#include <maptk/core/track_set.h>
#include <maptk/core/track_set_io.h>
#include <maptk/core/local_geo_cs.h>
#include <maptk/core/ins_data_io.h>
#include <maptk/core/landmark_map_io.h>
#include <maptk/core/camera_io.h>
#include <maptk/core/metrics.h>
#include <maptk/core/algo/bundle_adjust.h>
#include <maptk/core/algo/estimate_similarity_transform.h>
#include <maptk/core/algo/triangulate_landmarks.h>
#include <maptk/core/algo/geo_map.h>
#include <maptk/core/config_block.h>
#include <maptk/core/config_block_io.h>
#include <maptk/core/transform.h>
#include <maptk/core/types.h>

#include <boost/foreach.hpp>

#include <boost/filesystem.hpp>
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/value_semantic.hpp>
#include <boost/program_options/variables_map.hpp>


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
                    "This is optional, leave blank to ignore.");

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

  if (! config->has_value("input_pos_files"))
  {
    MAPTK_CONFIG_FAIL("No POS file specification given. This should either be an empty "
         "string or the path to a directory/file list of POS files.");
  }
  else if (config->get_value<std::string>("input_pos_files") != ""
           && !bfs::exists(config->get_value<std::string>("input_pos_files")))
  {
    MAPTK_CONFIG_FAIL("POS input path given, but doesn't point to an existing location.");
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
  if (!(   !config->has_value("st_estimator:type")
        || (config->get_value<std::string>("st_estimator:type") == "")
        || maptk::algo::estimate_similarity_transform::check_nested_algo_configuration("st_estimator", config)
       ))
  {
    MAPTK_CONFIG_FAIL("Failed config check in st_estimator algorithm.");
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


/// subsample a every Nth camera, where N is specfied by factor
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
  // the frames are "in order" in that tracking followed this list in this gived
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
  maptk::camera_map::map_camera_t cameras, pos_cameras;
  // if POS files are available, use them to initialize the cameras
  if( config->get_value<std::string>("input_pos_files") != "" )
  {
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
      if (filename2frame.size() != ins_map.size())
      {
        std::cerr << "Warning: Input POS file-set is sparse compared to input "
                  << "imagery! (not as many input POS files as there were input "
                  << "images)"
                  << std::endl;
      }

      cameras     = maptk::initialize_cameras_with_ins(ins_map, base_camera, local_cs);
      // Creating duplicate cameras structure
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

  maptk::camera_map_sptr cam_map(new maptk::simple_camera_map(cameras)),
                         orig_cam_map(new maptk::simple_camera_map(pos_cameras));

  std::cout << "initialized "<<cam_map->size()<<" cameras"<<std::endl;
  unsigned int cam_samp_rate = config->get_value<unsigned int>("camera_sample_rate");
  if(cam_samp_rate > 1)
  {
    cam_map = subsample_cameras(cam_map, cam_samp_rate);
    std::cout << "subsampled down to "<<cam_map->size()<<" cameras"<<std::endl;
  }

  //
  // Run bundle adjustment
  //
  double init_rmse = maptk::reprojection_rmse(cam_map->cameras(),
                                              lm_map->landmarks(),
                                              tracks->tracks());
  std::cout << "initial reprojection RMSE: " << init_rmse << std::endl;

  bundle_adjuster->optimize(cam_map, lm_map, tracks);

  double end_rmse = maptk::reprojection_rmse(cam_map->cameras(),
                                             lm_map->landmarks(),
                                             tracks->tracks());
  std::cout << "final reprojection RMSE: " << end_rmse << std::endl;


  //
  // Adjust cameras/landmarks based on input cameras/reference points
  //
  // If we were given POS files / reference points as input, compute a
  // similarity transform from the refined cameras to the POS file / reference
  // point cameras (via map structures). Then, apply the estimated transform to
  // the refined camera positions and landmarks.
  //
  if (orig_cam_map->size() > 0 && st_estimator)
  {
    std::cout << "Estimating and applying similarity transform to refined "
              << "cameras (from POS files)" << std::endl;
    maptk::similarity_d sim_transform = st_estimator->estimate_transform(cam_map, orig_cam_map);
    std::cout << "--> Estimated Transformation:" << std::endl
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
    std::string ply_file = config->get_value<std::string>("output_ply_file");
    write_ply_file(lm_map, ply_file);
  }

  //
  // Write the output POS files
  //
  if( config->has_value("output_pos_dir") )
  {
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
