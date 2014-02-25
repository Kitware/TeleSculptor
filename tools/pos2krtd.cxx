/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief POS file to KRTD conversion utility
 */

#include<iostream>
#include<fstream>
#include<exception>
#include<string>
#include<vector>

#include <maptk/modules.h>
#include <maptk/core/ins_data_io.h>
#include <maptk/core/camera_io.h>
#include <maptk/core/local_geo_cs.h>

#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>

namespace fs = boost::filesystem;


/// Convert a INS data to a camera
bool convert_ins2camera(const maptk::ins_data& ins,
                        maptk::local_geo_cs& cs,
                        maptk::camera_d& cam)
{
  if( cs.utm_origin_zone() < 0 )
  {
    std::cout << "lat: "<<ins.lat<<" lon: "<<ins.lon<<std::endl;
    cs.set_utm_origin_zone(cs.geo_map_algo()->latlon_zone(ins.lat, ins.lon));
    std::cout << "using zone "<< cs.utm_origin_zone() <<std::endl;
  }

  cs.update_camera(ins, cam);
  return true;
}


/// Convert a POS file to a KRTD file
bool convert_pos2krtd(const std::string& pos_filename,
                      const std::string& krtd_filename,
                      maptk::local_geo_cs& cs,
                      maptk::camera_d base_camera)
{
  maptk::ins_data ins;
  ins = maptk::read_pos_file(pos_filename);
  if ( !convert_ins2camera(ins, cs, base_camera) )
  {
    return false;
  }
  maptk::write_krtd_file(base_camera, krtd_filename);
  return true;
}


/// Convert a directory of POS file to a directory of KRTD files
bool convert_pos2krtd_dir(const fs::path& pos_dir,
                          const fs::path& krtd_dir,
                          maptk::local_geo_cs& cs,
                          maptk::camera_d base_camera)
{
  fs::directory_iterator it(pos_dir), eod;
  std::map<maptk::frame_id_t, maptk::ins_data> ins_map;
  std::vector<std::string> krtd_filenames;
  BOOST_FOREACH(fs::path const &p, std::make_pair(it, eod))
  {
    fs::path krtd_filename = krtd_dir / (basename(p) + ".krtd");
    std::cout << "processing "<< p <<" --> " << krtd_filename<< std::endl;
    maptk::frame_id_t frame = static_cast<maptk::frame_id_t>(krtd_filenames.size());
    ins_map[frame] = maptk::read_pos_file(p.string());
    krtd_filenames.push_back(krtd_filename.string());
  }

  std::map<maptk::frame_id_t, maptk::camera_sptr> cam_map;
  cam_map = maptk::initialize_cameras_with_ins(ins_map, base_camera, cs);

  typedef std::map<maptk::frame_id_t, maptk::camera_sptr>::value_type cam_map_val_t;
  BOOST_FOREACH(cam_map_val_t const &p, cam_map)
  {
    maptk::camera_d* cam = dynamic_cast<maptk::camera_d*>(p.second.get());
    maptk::write_krtd_file(*cam, krtd_filenames[p.first]);
  }

  maptk::vector_3d origin = cs.utm_origin();
  std::cout << "using local UTM origin at "<<origin[0] <<", "<<origin[1]
            <<", zone "<<cs.utm_origin_zone() <<std::endl;
  return true;
}


static int maptk_main(int argc, char const* argv[])
{
  // register the algorithms in the various modules for dynamic look-up
  maptk::register_modules();

  if( argc != 3 )
  {
    std::cerr << "Usage: "<<argv[0] <<" file.pos file.krtd"<<std::endl;
    return EXIT_FAILURE;
  }

  maptk::algo::geo_map_sptr geo_mapper = maptk::algo::geo_map::create("proj");
  if( !geo_mapper )
  {
    std::cerr << "No geo_map module available" << std::endl;
    return EXIT_FAILURE;
  }

  maptk::local_geo_cs local_cs(geo_mapper);

  if( fs::is_directory(argv[1]) )
  {
    std::cout << "processing "<<argv[1]<<" as a directory of POS files" << std::endl;
    if( !fs::exists(argv[2]) )
    {
      if( !fs::create_directory(argv[2]) )
      {
        std::cerr << "Unable to create output directory: " << argv[2] << std::endl;
        return EXIT_FAILURE;
      }
    }
    else if( !fs::is_directory(argv[2]) )
    {
      std::cerr << argv[2] <<" exists but is not a directory." << std::endl;
      return EXIT_FAILURE;
    }
    if( !convert_pos2krtd_dir(argv[1], argv[2], local_cs, maptk::camera_d()) )
    {
      return EXIT_FAILURE;
    }
  }
  // otherwise treat input as a single file
  else if( !convert_pos2krtd(argv[1], argv[2], local_cs, maptk::camera_d()) )
  {
    return EXIT_FAILURE;
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
