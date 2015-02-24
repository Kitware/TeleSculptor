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
 * \brief C Interface to maptk::track definition
 */

#ifndef MAPTK_C_TRACK_H_
#define MAPTK_C_TRACK_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>

#include <maptk/c/config.h>
#include <maptk/c/error_handle.h>


/// Opaque structure for maptk::track instances
typedef struct maptk_track_s maptk_track_t;


/// Create a new track
MAPTK_C_EXPORT
maptk_track_t*
maptk_track_new();


/// Destroy a MAPTK track pointer
MAPTK_C_EXPORT
void
maptk_track_destroy( maptk_track_t *track,
                     maptk_error_handle_t *eh );


/// Get the number of states in the track
MAPTK_C_EXPORT
size_t
maptk_track_size( maptk_track_t *track,
                  maptk_error_handle_t *eh );


/// Return whether or not this track has any states
MAPTK_C_EXPORT
bool
maptk_track_empty( maptk_track_t *track,
                   maptk_error_handle_t *eh );


#ifdef __cplusplus
}
#endif

#endif // MAPTK_C_TRACK_H_
