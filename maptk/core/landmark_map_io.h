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
 * \brief File IO functions for a \ref maptk::landmark_map
 *
 * Uses the PLY file format
 */

#ifndef MAPTK_LANDMARK_MAP_IO_H_
#define MAPTK_LANDMARK_MAP_IO_H_

#include "core_config.h"

#include "landmark_map.h"

namespace maptk
{


/// Output the given \c landmark_map object to the specified PLY file path
/**
 * If a file exists at the target location, it will be overwritten. If the
 * containing directory of the given path does not exist, it will be created
 * before the file is opened for writing.
 *
 * \throws file_write_exception
 *    Thrown when something prevents output of the file.
 * \throws boost::filesystem::filesystem:error
 *    Thrown when an underlying boost::filesystem call fails for system
 *    reasons.
 *
 * \param landmarks The \c landmark_map object to output.
 * \param file_path The path to output the file to.
 */
void
MAPTK_CORE_EXPORT
write_ply_file(landmark_map_sptr const& landmarks,
               path_t const& file_path);


/// Load a given \c landmark_map object from the specified PLY file path
/**
 * This function does not read all ply files, only ply files which have
 * been output by the landmark write ply function.
 *
 * \throws file_read_exception
 *    Thrown when something prevents input of the file.
 * \throws boost::filesystem::filesystem:error
 *    Thrown when an underlying boost::filesystem call fails for system
 *    reasons.
 *
 * \param file_path The path to output the file to.
 */
landmark_map_sptr
MAPTK_CORE_EXPORT
read_ply_file(path_t const& file_path);

}

#endif // MAPTK_LANDMARK_MAP_IO_H_
