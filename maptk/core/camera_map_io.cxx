/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
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


camera_map_sptr
read_krtd_files(std::vector<path_t> const& img_files, path_t const& dir)
{
  if( !boost::filesystem::exists(dir) )
  {
    return camera_map_sptr();
  }

  std::vector<path_t> files_to_read;
  camera_map::map_camera_t cameras;

  for( unsigned i = 0; i < img_files.size(); i++ )
  {
    files_to_read.push_back( dir / img_files[i].stem() );
    std::string adj_path = files_to_read[i].string();
    files_to_read[i] = boost::filesystem::path( adj_path.append( ".krtd" ) );
  }

  for( frame_id_t fid = 0; fid < files_to_read.size(); fid++ )
  {
    try
    {
      camera_d new_camera = read_krtd_file( files_to_read[fid] );
      cameras[fid] = camera_sptr( new camera_d( new_camera ) );
    }
    catch( file_not_found_exception )
    {
      continue;
    }
  }

  if( cameras.empty() )
  {
    return camera_map_sptr();
  }

  return camera_map_sptr( new simple_camera_map( cameras ) );
}


} // end namespace maptk
