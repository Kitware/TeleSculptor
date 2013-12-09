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
#include <maptk/core/algo/image_io.h>
#include <maptk/core/algo/detect_features.h>
#include <maptk/core/algo/extract_descriptors.h>

static int maptk_main(int argc, char const* argv[])
{
  // register the algorithms in the various modules for dynamic look-up
  maptk::register_modules();

  if( argc != 2 )
  {
    std::cerr << "Usage: "<<argv[0] <<" image_list.txt"<<std::endl;
    return EXIT_FAILURE;
  }

  std::ifstream ifs(argv[1]);
  if (!ifs)
  {
    std::cerr << "Error: Could not open image list "<<argv[1]<<std::endl;
    return EXIT_FAILURE;
  }
  std::vector<std::string> files;
  for (std::string line; std::getline(ifs,line); )
  {
    files.push_back(line);
  }


  typedef boost::shared_ptr<maptk::algo::image_io> image_io_sptr;
  image_io_sptr image_reader = maptk::algo::image_io::create("ocv");

  if( !image_reader )
  {
    std::cerr << "No image I/O module available" << std::endl;
    return EXIT_FAILURE;
  }

  for(unsigned i=0; i<files.size(); ++i)
  {
    std::cout << "processing frame "<<i<<": "<<files[i]<<std::endl;
    maptk::image_container_sptr img = image_reader->load(files[i]);
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
