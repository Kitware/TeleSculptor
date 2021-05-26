// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

/**
 * \file
 * \brief Helper functions shared by tools
 */

#ifndef MAPTK_TOOL_COMMON_H_
#define MAPTK_TOOL_COMMON_H_

#include <cstdio>

#include <vital/exceptions.h>
#include <vital/io/camera_io.h>
#include <vital/logger/logger.h>
#include <vital/types/camera_map.h>
#include <vital/vital_types.h>

#include <kwiversys/Directory.hxx>
#include <kwiversys/SystemTools.hxx>

namespace kwiver {
namespace maptk {

/// Return a sorted list of files in a directory
std::vector< kwiver::vital::path_t >
files_in_dir(kwiver::vital::path_t const& vdir)
{
  using namespace kwiver;
  std::vector< vital::path_t > files;

  kwiversys::Directory dir;
  if ( 0 == dir.Load( vdir ) )
  {
    vital::logger_handle_t main_logger( vital::get_logger( "files_in_dir" ) );
    LOG_WARN(main_logger, "Could not access directory \"" << vdir << "\"");
    return files;
  }

  unsigned long num_files = dir.GetNumberOfFiles();
  for ( unsigned long i = 0; i < num_files; i++)
  {
    files.push_back( vdir + '/' + dir.GetFile( i ) );
  }

  std::sort( files.begin(), files.end() );
  return files;
}

// Load input KRTD cameras from a directory, matching against the given image
// filename map.
kwiver::vital::camera_map::map_camera_t
load_input_cameras_krtd(std::string const& krtd_dir,
                        std::map<kwiver::vital::frame_id_t, std::string> const& basename_map)
{
  kwiver::vital::camera_map::map_camera_t krtd_cams;
  for (auto p : basename_map)
  {
    std::string krtd_filename = krtd_dir + '/' + p.second + ".krtd";
    try
    {
      kwiver::vital::camera_sptr cam = kwiver::vital::read_krtd_file(krtd_filename);
      krtd_cams[p.first] = cam;
    }
    catch(kwiver::vital::file_not_found_exception)
    {
    }
  }

  // if krtd_map is empty, then there were no input krtd files that matched
  // input imagery.
  if (krtd_cams.empty())
  {
    vital::logger_handle_t logger( vital::get_logger( "load_input_cameras_krtd" ) );
    LOG_ERROR(logger, "No KRTD files from input set match input image "
                      << "frames. Check KRTD input files!");
    return kwiver::vital::camera_map::map_camera_t();
  }

  // Warning if loaded KRTD camera set is sparse compared to input imagery
  // TODO: generated interpolated cameras for missing KRTD files.
  if (basename_map.size() != krtd_cams.size())
  {
    vital::logger_handle_t logger( vital::get_logger( "load_input_cameras_krtd" ) );
    LOG_WARN(logger, "Input KRTD camera set is sparse compared to input "
                     << "imagery! (there wasn't a matching KRTD input file for "
                     << "every input image file)");
  }
  return krtd_cams;
}

} // end namespace maptk
} // end namespace kwiver

#endif
