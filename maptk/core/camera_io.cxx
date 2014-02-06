/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include "camera_io.h"

#include <fstream>

#include <maptk/core/exceptions.h>

#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>


/**
 * \file
 * \brief Implementation of file IO functions for a \ref maptk::camera
 *
 * File format is the KRTD file.
 */

namespace maptk
{


/// Read in a KRTD file, producing a camera object
camera_d
read_krtd_file(path_t const& file_path)
{
  // Check that file exists
  if( ! boost::filesystem::exists(file_path) )
  {
    throw file_not_found_exception(file_path, "File does not exist.");
  }
  else if ( ! boost::filesystem::is_regular_file(file_path) )
  {
    throw file_not_found_exception(file_path, "Path given doesn't point to "
                                              "a regular file!");
  }

  // Reading in input file data
  std::ifstream input_stream(file_path.c_str(), std::fstream::in);
  if( ! input_stream )
  {
    throw file_not_read_exception(file_path, "Could not open file at given "
                                             "path.");
  }

  // Read the file
  camera_d cam;
  input_stream >> cam;
  return cam;
}


/// Output the given \c camera object to the specified file path
void
write_krtd_file(camera const& cam,
                path_t const& file_path)
{
  namespace bfs = boost::filesystem;

  // If the given path is a directory, we obviously can't write to it.
  if(bfs::is_directory(file_path))
  {
    throw file_write_exception(file_path, "Path given is a directory, "
                                          "can not write file.");
  }

  // Check that the directory of the given filepath exists, creating necessary
  // directories where needed.
  path_t parent_dir = bfs::absolute(file_path.parent_path());
  if(!bfs::is_directory(parent_dir))
  {
    if(!bfs::create_directories(parent_dir))
    {
      throw file_write_exception(parent_dir, "Attempted directory creation, "
                                             "but no directory created! No "
                                             "idea what happened here...");
    }
  }


  // open output file and write the ins_data
  std::ofstream ofile(file_path.c_str());
  ofile << cam;
  ofile.close();
}

}
