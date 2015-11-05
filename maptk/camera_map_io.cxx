/*ckwg +29
 * Copyright 2014 by Kitware, Inc.
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
 * \brief Implementation of camera map io functions
 */

#include "camera_map_io.h"
#include "camera_io.h"
#include "exceptions.h"

#include <boost/filesystem.hpp>

namespace maptk
{


/// Load a camera map from krtd files stored in a directory.
camera_map_sptr
read_krtd_files(std::vector<path_t> const& img_files, path_t const& dir)
{
  if( !boost::filesystem::exists( dir ) )
  {
    throw path_not_exists( dir );
  }

  camera_map::map_camera_t cameras;

  for( frame_id_t fid = 0; fid < img_files.size(); ++fid )
  {
    try
    {
      camera_d new_camera = read_krtd_file( img_files[fid], dir );
      cameras[fid] = camera_sptr( new camera_d( new_camera ) );
    }
    catch( file_not_found_exception )
    {
      continue;
    }
  }

  if( cameras.empty() )
  {
    throw invalid_data( "No krtd files found" );
  }

  return camera_map_sptr( new simple_camera_map( cameras ) );
}


} // end namespace maptk
