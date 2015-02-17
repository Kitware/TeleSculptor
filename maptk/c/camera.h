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
 * \brief C Interface to \p maptk::camera class
 */

#ifndef MAPTK_C_CAMERA_H_
#define MAPTK_C_CAMERA_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <maptk/c/config.h>
#include <maptk/c/error_handle.h>


/// Opaque structure to a maptk::camera class
typedef struct maptk_camera_s maptk_camera_t;


/// TODO: New and member methods

/// Destroy a maptk_camera_t instance
/**
 * The given might not refer to a valid camera instance, causing the error
 * handle to be populated (code -1).
 */
MAPTK_C_EXPORT
void maptk_camera_destroy( maptk_camera_t *cam,
                           maptk_error_handle_t *eh );


/// Read in a KRTD file, producing a new maptk::camera object
MAPTK_C_EXPORT
maptk_camera_t* maptk_camera_read_krtd_file( char const *filepath,
                                             maptk_error_handle_t *eh );


/// Output the given maptk_camera_t object to the specified file path
MAPTK_C_EXPORT
void maptk_camera_write_krtd_file( maptk_camera_t *cam,
                                   char const *filepath,
                                   maptk_error_handle_t *eh );


#ifdef __cplusplus
}
#endif

#endif // MAPTK_C_CAMERA_H_
