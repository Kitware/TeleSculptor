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
 * \brief C interface to maptk::image_container class
 */

#ifndef MAPTK_C_IMAGE_CONTAINER_H_
#define MAPTK_C_IMAGE_CONTAINER_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stddef.h>

#include <maptk/c/config.h>
#include <maptk/c/error_handle.h>
#include <maptk/c/image.h>


/// MAPTK Image opaque structure
typedef struct maptk_image_container_s maptk_image_container_t;


/// Create a new, simple image container around an image
MAPTK_C_EXPORT
maptk_image_container_t* maptk_image_container_new_simple( maptk_image_t *img );


/// Destroy a maptk_image_container_t instance
MAPTK_C_EXPORT
void maptk_image_container_destroy( maptk_image_container_t *img_container,
                                    maptk_error_handle_t *eh );


/// Get the size in bytes of an image container
/**
 * Size includes all allocated image memory, which could be larger than
 * the product of width, height and depth.
 */
MAPTK_C_EXPORT
size_t maptk_image_container_size( maptk_image_container_t *img_c );


/// Get the width of the given image in pixels
MAPTK_C_EXPORT
size_t maptk_image_container_width( maptk_image_container_t *img_c );


/// Get the height of the given image in pixels
MAPTK_C_EXPORT
size_t maptk_image_container_height( maptk_image_container_t *img_c );


/// Get the depth (number of channels) of the image
MAPTK_C_EXPORT
size_t maptk_image_container_depth( maptk_image_container_t *img_c );


/// Get the in-memory image class to access data
MAPTK_C_EXPORT
maptk_image_t* maptk_image_container_get_image( maptk_image_container_t *img_c );


#ifdef __cplusplus
}
#endif

#endif // MAPTK_C_IMAGE_CONTAINER_H_
