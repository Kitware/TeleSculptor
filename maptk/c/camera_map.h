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
 * \brief C interface for maptk::camera_map
 */

#ifndef MAPTK_C_CAMERA_MAP_H_
#define MAPTK_C_CAMERA_MAP_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stddef.h>

#include <maptk/c/camera.h>
#include <maptk/c/error_handle.h>


/// Opaque structure for maptk::camera_map class
typedef struct maptk_camera_map_s maptk_camera_map_t;


/// New, simple camera map
/**
 * Given a two parallel arrays of frame number and cameras, create a new
 * camera map.
 *
 * If either array is NULL or if length is zero, the returned camera_map will
 * be empty.
 */
MAPTK_C_EXPORT
maptk_camera_map_t* maptk_camera_map_new( size_t length,
                                          unsigned int *frame_numbers,
                                          maptk_camera_t **cameras );


/// Destroy the given camera_map
MAPTK_C_EXPORT
void maptk_camera_map_destroy( maptk_camera_map_t *cam_map,
                               maptk_error_handle_t *eh );


/// Return the number of cameras in the map
MAPTK_C_EXPORT
size_t maptk_camera_map_size( maptk_camera_map_t *cam_map,
                              maptk_error_handle_t *eh );


/// Set pointers to parallel arrays of frame numers and camera instances
MAPTK_C_EXPORT
void maptk_camera_map_get_map( maptk_camera_map_t *cam_map,
                               size_t *length,
                               unsigned int **frame_numbers,
                               maptk_camera_t ***cameras,
                               maptk_error_handle_t *eh );

// TODO Free method for allocated frame/camera parallel arrays
/// Free paired frame-to-camera mapping arrays
//void maptk_camera_map_free_mapping_array( size_t length,
//                                          unsigned int *frame_numbers,
//                                          maptk_camera_t **cameras );


#ifdef __cplusplus
}
#endif

#endif // MAPTK_C_CAMERA_MAP_H_
