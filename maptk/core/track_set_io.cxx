/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include "track_set_io.h"

#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <map>

#include <maptk/core/exceptions.h>

#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>


namespace maptk
{


/// Read in a track file, producing a track_set
track_set_sptr
read_track_file(path_t const& file_path)
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
  std::vector<track_sptr> tracks;
  std::map<track_id_t, track_sptr> track_map;
  for (std::string line; std::getline(input_stream, line); )
  {
    track_id_t tid;
    frame_id_t fid;
    feature_d feat;
    std::stringstream ss(line);
    ss >> tid >> fid >> feat;

    track_sptr t;
    std::map<track_id_t, track_sptr>::const_iterator it = track_map.find(tid);
    if(it == track_map.end())
    {
      t = track_sptr(new track);
      t->set_id(tid);
      tracks.push_back(t);
      track_map[tid] = t;
    }
    else
    {
      t = it->second;
    }
    t->append(track::track_state(fid, feature_sptr(new feature_d(feat)),
                                 descriptor_sptr()));
  }

  return track_set_sptr(new simple_track_set(tracks));
}


/// Output the given \c track_set object to the specified file path
void
write_track_file(track_set_sptr const& tracks,
                 path_t const& file_path)
{
  namespace bfs = boost::filesystem;

  // If the track set is empty, throw
  if(!tracks || tracks->size() == 0)
  {
    throw file_write_exception(file_path, "No tracks in the given "
                                          "track_set!");
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
  std::vector<maptk::track_sptr> trks = tracks->tracks();
  BOOST_FOREACH(maptk::track_sptr t, trks)
  {
    typedef maptk::track::history_const_itr state_itr;
    for (state_itr si = t->begin(); si != t->end(); ++si)
    {
      ofile << t->id() << " " << si->frame_id << " " << *si->feat << "\n";
    }
  }
  ofile.close();
}

}
