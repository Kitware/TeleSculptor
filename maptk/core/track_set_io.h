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
 * \brief File IO functions for a \ref maptk::track_set
 *
 * \todo Describe format here.
 */

#ifndef MAPTK_TRACK_SET_IO_H_
#define MAPTK_TRACK_SET_IO_H_

#include "core_config.h"

#include "track_set.h"


namespace maptk
{

/// Read in a track file, producing a track_set
/**
 * \note The simple track file format does not represent all data within
 *       a track.  It only stores the minimal subset of data needed later
 *       to run sparse bundle adjustment.
 * \throws file_not_found_exception
 *    Thrown when the file could not be found on the file system.
 * \throws file_not_read_exception
 *    Thrown when the file could not be read or parsed for whatever reason.
 * \throws boost::filesystem::filesystem_error
 *    Boost exception thrown if something goes wrong with the underlying file
 *    read.
 *
 * \param file_path   The path to the file to read in.
 * \return A \c track_set object representing the contents of the read-in file.
 */
track_set_sptr
MAPTK_CORE_EXPORT
read_track_file(path_t const& file_path);

/// Output the given \c track_set object to the specified file path
/**
 * If a file exists at the target location, it will be overwritten. If the
 * containing directory of the given path does not exist, it will be created
 * before the file is opened for writing.
 *
 * \note The simple track file format does not represent all data within
 *       a track.  It only stores the minimal subset of data needed later
 *       to run sparse bundle adjustment.
 * \throws file_write_exception
 *    Thrown when something prevents output of the file.
 * \throws boost::filesystem::filesystem:error
 *    Thrown when an underlying boost::filesystem call failes for system
 *    reasons.
 *
 * \param tracks    The \c track_set object to output.
 * \param file_path The path to output the file to.
 */
void
MAPTK_CORE_EXPORT
write_track_file(track_set_sptr const& tracks,
                 path_t const& file_path);

}

#endif // MAPTK_TRACK_SET_IO_H_
