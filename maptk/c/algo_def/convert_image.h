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
 * \brief C Interface to convert_image algorithm definition
 */

#ifndef MAPTK_C_ALGO_DEF_CONVERT_IMAGE_H_
#define MAPTK_C_ALGO_DEF_CONVERT_IMAGE_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <maptk/c/algorithm.h>
#include <maptk/c/config.h>
#include <maptk/c/image_container.h>


/// Declare common type-specific functions
DECLARE_COMMON_ALGO_API( convert_image );


/// Convert image base type
/**
 * Returns new image container with a converted underlying representation
 * based on the implementation configured.
 *
 * \param algo Opaque pointer to algorithm instance.
 * \param ic Image container with the image data to convert.
 * \param eh Error handle instance.
 * \return New image container with the converted underlying data.
 */
MAPTK_C_EXPORT
maptk_image_container_t*
maptk_algorithm_convert_image_convert( maptk_algorithm_t *algo,
                                       maptk_image_container_t *ic,
                                       maptk_error_handle_t *eh );


#ifdef __cplusplus
}
#endif

#endif // MAPTK_C_ALGO_DEF_CONVERT_IMAGE_H_
