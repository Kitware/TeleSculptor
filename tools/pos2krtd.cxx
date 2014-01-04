/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include<iostream>
#include<fstream>
#include<exception>
#include<string>
#include<vector>

#include <maptk/modules.h>
#include <maptk/core/local_geo_cs.h>

#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>

namespace fs = boost::filesystem;


/// Read a POS file from disk into an INS data structure
bool read_pos_file(const std::string& pos_filename,
                   maptk::ins_data& ins)
{
  std::ifstream ifs(pos_filename.c_str());
  if (!ifs)
  {
    std::cerr << "Error: Could not open POS file "<<pos_filename<<std::endl;
    return false;
  }

  ifs >> ins;
  return ! ifs.fail();
}


/// Write a camera to a KRTD file on disk
bool write_krtd_file(const std::string& krtd_filename,
                     const maptk::camera_d& cam)
{
  std::ofstream ofs(krtd_filename.c_str());
  if (!ofs)
  {
    std::cerr << "Error: Could not open KRTD file "<<krtd_filename<<std::endl;
    return false;
  }

  ofs << cam;
  return ! ofs.fail();
}


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
  return read_pos_file(pos_filename, ins) &&
         convert_ins2camera(ins, cs, base_camera) &&
         write_krtd_file(krtd_filename, base_camera);
}


/// Convert a directory of POS file to a directory of KRTD files
bool convert_pos2krtd_dir(const fs::path& pos_dir,
                          const fs::path& krtd_dir,
                          maptk::local_geo_cs& cs,
                          maptk::camera_d base_camera)
{
  fs::directory_iterator it(pos_dir), eod;
  std::vector<maptk::camera_d> cameras;
  std::vector<std::string> krtd_filenames;
  BOOST_FOREACH(fs::path const &p, std::make_pair(it, eod))
  {
    fs::path krtd_filename = krtd_dir / (basename(p) + ".krtd");
    std::cout << "processing "<< p <<" --> " << krtd_filename<< std::endl;
    maptk::ins_data ins;
    if ( !read_pos_file(p.string(), ins) )
    {
      return false;
    }
    convert_ins2camera(ins, cs, base_camera);
    cameras.push_back(base_camera);
    krtd_filenames.push_back(krtd_filename.string());
  }

  maptk::vector_3d mean(0,0,0);
  BOOST_FOREACH(const maptk::camera_d& cam, cameras)
  {
    mean += cam.center();
  }
  mean /= cameras.size();
  // only use the mean easting and northing
  mean[2] = 0.0;

  for(unsigned int i=0; i<cameras.size(); ++i)
  {
    maptk::camera_d& cam = cameras[i];
    cam.set_center(cam.center() - mean);

    if( !write_krtd_file(krtd_filenames[i], cam) )
    {
      return false;
    }
  }

  std::cout << "using local UTM origin at "<<mean[0] <<", "<<mean[1]
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
