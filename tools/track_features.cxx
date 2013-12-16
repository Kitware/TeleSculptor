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
#include <maptk/core/algo/match_features.h>
#include <maptk/core/algo/track_features.h>

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

  namespace alg = maptk::algo;
  alg::image_io_sptr image_reader = alg::image_io::create("ocv");
  alg::detect_features_sptr feature_detector = alg::detect_features::create("ocv");
  alg::extract_descriptors_sptr descriptor_extractor = alg::extract_descriptors::create("ocv");
  alg::match_features_sptr feature_matcher = alg::match_features::create("ocv");
  alg::track_features_sptr feature_tracker = alg::track_features::create("simple");

  if( !image_reader )
  {
    std::cerr << "No image I/O module available" << std::endl;
    return EXIT_FAILURE;
  }
  if( !feature_detector )
  {
    std::cerr << "No feature detector module available" << std::endl;
    return EXIT_FAILURE;
  }
  if( !descriptor_extractor )
  {
    std::cerr << "No descriptor extractor module available" << std::endl;
    return EXIT_FAILURE;
  }
  if( !feature_matcher )
  {
    std::cerr << "No feature matcher module available" << std::endl;
    return EXIT_FAILURE;
  }
  if( !feature_tracker )
  {
    std::cerr << "No feature_tracker module available" << std::endl;
    return EXIT_FAILURE;
  }

  feature_tracker->set_feature_detector(feature_detector);
  feature_tracker->set_descriptor_extractor(descriptor_extractor);
  feature_tracker->set_feature_matcher(feature_matcher);

  maptk::track_set_sptr tracks;
  for(unsigned i=0; i<files.size(); ++i)
  {
    std::cout << "processing frame "<<i<<": "<<files[i]<<std::endl;
    maptk::image_container_sptr img = image_reader->load(files[i]);
    tracks = feature_tracker->track(tracks, i, img);
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
