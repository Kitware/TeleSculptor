/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include "landmark_map_io.h"

#include <fstream>
#include <iostream>

#include <maptk/core/exceptions.h>

#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>


namespace maptk
{


/// Output the given \c landmark_map object to the specified PLY file path
void
write_ply_file(landmark_map_sptr const& landmarks,
               path_t const& file_path)
{
  namespace bfs = boost::filesystem;

  // If the landmark map is empty, throw
  if(!landmarks || landmarks->size() == 0)
  {
    throw file_write_exception(file_path, "No landmarks in the given "
                                          "landmark map!");
  }

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


  // open output file and write the tracks
  std::ofstream ofile(file_path.c_str());
  // write the PLY header
  ofile << "ply\n"
           "format ascii 1.0\n"
           "comment written by MAPTK\n"
           "element vertex "<<landmarks->size()<<"\n"
           "property float x\n"
           "property float y\n"
           "property float z\n"
           "property uint track_id\n"
           "end_header\n";

  landmark_map::map_landmark_t lm_map = landmarks->landmarks();
  typedef  landmark_map::map_landmark_t::value_type lm_map_val_t;
  BOOST_FOREACH(lm_map_val_t const& p, lm_map)
  {
    vector_3d loc = p.second->loc();
    ofile << loc.x() << " " << loc.y() << " " << loc.z()
          << " " << p.first << "\n";
  }
  ofile.close();
}

}
