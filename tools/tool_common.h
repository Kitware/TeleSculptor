/*ckwg +29
 * Copyright 2017 by Kitware, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither name of Kitware, Inc. nor the names of any contributors may be used
 *    to endorse or promote products derived from this software without specific
 *    prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHORS OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

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
