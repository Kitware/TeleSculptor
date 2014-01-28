/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include<iostream>
#include<fstream>
#include<exception>
#include<string>
#include<vector>

#include <maptk/modules.h>

#include <maptk/core/track_set.h>
#include <maptk/core/track_set_io.h>
#include <maptk/core/local_geo_cs.h>
#include <maptk/core/ins_data_io.h>
#include <maptk/core/metrics.h>
#include <maptk/core/algo/bundle_adjust.h>
#include <maptk/core/algo/geo_map.h>
#include <maptk/core/config_block.h>
#include <maptk/core/config_block_io.h>
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

  config->set_value("track_file", "",
                    "Path an input file containing feature tracks");

  config->set_value("pos_files", "",
                    "A directory containing the POS files, or a text file"
                    "containing a newline-separated list of POS files. "
                    "This is optional, leave blank to ignore.");

  algo::bundle_adjust::get_nested_algo_configuration("bundle_adjuster", config,
                                                     algo::bundle_adjust_sptr());
  algo::geo_map::get_nested_algo_configuration("geo_mapper", config,
                                               algo::geo_map_sptr());

  return config;
}


static bool check_config(maptk::config_block_sptr config)
{
  return (
         config->has_value("track_file")
      && bfs::exists(maptk::path_t(config->get_value<std::string>("track_file")))
      && (  !config->has_value("pos_files")
         || bfs::exists(maptk::path_t(config->get_value<std::string>("pos_files"))) )
      && maptk::algo::bundle_adjust::check_nested_algo_configuration("bundle_adjuster", config)
      && maptk::algo::geo_map::check_nested_algo_configuration("geo_mapper", config)
      );
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
  maptk::config_block_sptr config = default_config();
  algo::bundle_adjust_sptr bundle_adjuster;
  algo::geo_map_sptr geo_mapper;

  // If -c/--config given, read in confg file, merge in with default just generated
  if(vm.count("config"))
  {
    //std::cerr << "[DEBUG] Given config file: " << vm["config"].as<maptk::path_t>() << std::endl;
    config->merge_config(maptk::read_config_file(vm["config"].as<maptk::path_t>()));
  }

  //std::cerr << "[DEBUG] Config BEFORE set:" << std::endl;
  //print_config(config);

  algo::bundle_adjust::set_nested_algo_configuration("bundle_adjuster", config, bundle_adjuster);
  algo::bundle_adjust::get_nested_algo_configuration("bundle_adjuster", config, bundle_adjuster);
  algo::geo_map::set_nested_algo_configuration("geo_mapper", config, geo_mapper);
  algo::geo_map::get_nested_algo_configuration("geo_mapper", config, geo_mapper);

  //std::cerr << "[DEBUG] Config AFTER set:" << std::endl;
  //print_config(config);

  bool valid_config = check_config(config);

  if(vm.count("output-config"))
  {
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

  // Read the track file
  std::string track_file = config->get_value<std::string>("track_file");
  maptk::track_set_sptr tracks = maptk::read_track_file(track_file);

  // Create the local coordinate system
  maptk::local_geo_cs local_cs(geo_mapper);
  maptk::camera_d base_camera;

  std::map<maptk::frame_id_t, maptk::camera_sptr> cameras;
  // if POS files are available, use them to initialize the cameras
  if( config->has_value("pos_files") )
  {
    std::string pos_files = config->get_value<std::string>("pos_files");
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
        std::cerr << "Error: Could not POS file list \""<<pos_files<<"\""<<std::endl;
        return EXIT_FAILURE;
      }

      for (std::string line; std::getline(ifs,line); )
      {
        files.push_back(line);
      }
    }

    std::map<maptk::frame_id_t, maptk::ins_data> ins_map;
    maptk::frame_id_t frame = 0;
    BOOST_FOREACH(const bfs::path& fpath, files)
    {
      ins_map[frame++] = maptk::read_pos_file(fpath);
    }
    cameras = maptk::initialize_cameras_with_ins(ins_map, base_camera, local_cs);
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

  // Initialize all landmarks to the origin
  std::set<maptk::track_id_t> track_ids = tracks->all_track_ids();
  maptk::landmark_map::map_landmark_t landmarks;
  BOOST_FOREACH(const maptk::track_id_t& tid, track_ids)
  {
    maptk::landmark_sptr lm(new maptk::landmark_d(maptk::vector_3d(0,0,0)));
    landmarks[static_cast<maptk::landmark_id_t>(tid)] = lm;
  }
  // TODO triangulate landmarks if initial cameras are valid

  // Run bundle adjustment
  maptk::camera_map_sptr cam_map(new maptk::simple_camera_map(cameras));
  maptk::landmark_map_sptr lm_map(new maptk::simple_landmark_map(landmarks));

  double init_rmse = maptk::reprojection_rmse(cam_map->cameras(),
                                              lm_map->landmarks(),
                                              tracks->tracks());
  std::cout << "initial reprojection RMSE: " << init_rmse << std::endl;

  bundle_adjuster->optimize(cam_map, lm_map, tracks);

  double end_rmse = maptk::reprojection_rmse(cam_map->cameras(),
                                             lm_map->landmarks(),
                                             tracks->tracks());
  std::cout << "final reprojection RMSE: " << end_rmse << std::endl;

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
