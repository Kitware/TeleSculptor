/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
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
