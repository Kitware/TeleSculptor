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

#include <vital/vital_types.h>
#include <vital/logger/logger.h>
#include <vital/video_metadata/video_metadata.h>

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


/// Extract an image file basename from metadata and (if needed) frame number
/**
 * The purpose of this function is to provide a standard way to get a base file
 * name (no file extension) from metadata.  This is either the original image
 * file basename, if provided, or the video basename, if provided, with frame
 * number appended, or simply "frame%05d".
 */
std::string
basename_from_metadata(kwiver::vital::video_metadata const& md,
                       kwiver::vital::frame_id_t frame)
{
  typedef kwiversys::SystemTools  ST;

  std::string basename = "frame";
  if( md.has( kwiver::vital::VITAL_META_IMAGE_FILENAME ) )
  {
    std::string img_name = md.find( kwiver::vital::VITAL_META_IMAGE_FILENAME ).as_string();
    basename = ST::GetFilenameWithoutLastExtension( img_name );
  }
  else
  {
    if ( md.has( kwiver::vital::VITAL_META_VIDEO_FILENAME ) )
    {
      std::string vid_name = md.find( kwiver::vital::VITAL_META_VIDEO_FILENAME ).as_string();
      basename = ST::GetFilenameWithoutLastExtension( vid_name );
    }
    char frame_str[6];
    std::snprintf(frame_str, 6, "%05d", static_cast<int>(frame));
    basename += std::string(frame_str);
  }
  return basename;
}

} // end namespace maptk
} // end namespace kwiver


#endif
