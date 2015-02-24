/*ckwg +29
 * Copyright 2015 by Kitware, Inc.
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
 * \brief C interface implementation for maptk::camera_map
 */

#include "camera_map.h"

#include <boost/foreach.hpp>

#include <maptk/camera_map.h>

#include <maptk/c/camera.h>

#include <maptk/c/helpers/c_utils.h>
#include <maptk/c/helpers/camera.h>
#include <maptk/c/helpers/camera_map.h>


namespace maptk_c
{

SharedPointerCache< maptk::camera_map,
                    maptk_camera_map_t > CAM_MAP_SPTR_CACHE( "camera_map" );

}


/// New, simple camera map
maptk_camera_map_t* maptk_camera_map_new( size_t length,
                                          unsigned int *frame_numbers,
                                          maptk_camera_t **cameras )
{
  STANDARD_CATCH(
    "C::camera_map::new", 0,
    if( frame_numbers == 0 || cameras == 0 )
    {
      length = 0;
    }
    // Create std::map of the paired items given
    // This assumes that the sptr type defined in the cache type is the same as
    // the type defined in C++-land (which is should be?)
    maptk::camera_map::map_camera_t cmap;
    for( size_t i=0; i < length; i++ )
    {
      cmap[frame_numbers[i]] = maptk_c::CAMERA_SPTR_CACHE.get( cameras[i] );
    }
    maptk::camera_map_sptr cm_sptr( new maptk::simple_camera_map( cmap ) );
    maptk_c::CAM_MAP_SPTR_CACHE.store( cm_sptr );
    return reinterpret_cast<maptk_camera_map_t*>( cm_sptr.get() );
  );
  return 0;
}


/// Destroy the given camera_map
void maptk_camera_map_destroy( maptk_camera_map_t *cam_map,
                               maptk_error_handle_t *eh )
{
  STANDARD_CATCH(
    "C::camera_map::destroy", eh,
    maptk_c::CAM_MAP_SPTR_CACHE.erase( cam_map );
  );
}


/// Return the number of cameras in the map
size_t maptk_camera_map_size( maptk_camera_map_t *cam_map,
                              maptk_error_handle_t *eh )
{
  STANDARD_CATCH(
    "C::camera_map::size", eh,
    return maptk_c::CAM_MAP_SPTR_CACHE.get( cam_map )->size();
  );
  return 0;
}


/// Set pointers to parallel arrays of frame numers and camera instances
void maptk_camera_map_get_map( maptk_camera_map_t *cam_map,
                               size_t *length,
                               unsigned int **frame_numbers,
                               maptk_camera_t ***cameras,
                               maptk_error_handle_t *eh )
{
  STANDARD_CATCH(
    "C::camera_map::get_map", eh,
    maptk::camera_map::map_camera_t map_cams =
      maptk_c::CAM_MAP_SPTR_CACHE.get( cam_map )->cameras();
    *length = map_cams.size();
    *frame_numbers = (unsigned int*)malloc(sizeof(unsigned int) * *length);
    *cameras = (maptk_camera_t**)malloc(sizeof(maptk_camera_t*) * *length);
    size_t i=0;
    BOOST_FOREACH( maptk::camera_map::map_camera_t::value_type const &p, map_cams )
    {
      (*frame_numbers)[i] = p.first;
      maptk_c::CAMERA_SPTR_CACHE.store( p.second );
      (*cameras)[i] = reinterpret_cast<maptk_camera_t*>( p.second.get() );
      ++i;
    }
  );
}
