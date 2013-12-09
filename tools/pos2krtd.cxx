/*ckwg +5
 * Copyright 2013 by Kitware, Inc. All Rights Reserved. Please refer to
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

static int maptk_main(int argc, char const* argv[])
{
  // register the algorithms in the various modules for dynamic look-up
  maptk::register_modules();

  if( argc != 3 )
  {
    std::cerr << "Usage: "<<argv[0] <<" file.pos file.krtd"<<std::endl;
    return EXIT_FAILURE;
  }

  std::ifstream ifs(argv[1]);
  if (!ifs)
  {
    std::cerr << "Error: Could not open POS file "<<argv[1]<<std::endl;
    return EXIT_FAILURE;
  }

  std::ofstream ofs(argv[2]);
  if (!ofs)
  {
    std::cerr << "Error: Could not open KRTD file "<<argv[2]<<std::endl;
    return EXIT_FAILURE;
  }

  maptk::ins_data ins;
  ifs >> ins;

  maptk::algo::geo_map_sptr geo_mapper = maptk::algo::geo_map::create("proj");
  if( !geo_mapper )
  {
    std::cerr << "No geo_map module available" << std::endl;
    return EXIT_FAILURE;
  }

  maptk::local_geo_cs local_cs(geo_mapper);
  std::cout << "lat: "<<ins.lat<<" lon: "<<ins.lon<<std::endl;
  local_cs.set_utm_origin_zone(geo_mapper->latlon_zone(ins.lat, ins.lon));
  std::cout << "using zone "<< local_cs.utm_origin_zone() <<std::endl;

  maptk::camera_d cam;
  local_cs.update_camera(ins, cam);

  ofs << cam;

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
