/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_CAMERA_IO_H_
#define MAPTK_CAMERA_IO_H_

#include "core_config.h"
#include "types.h"

#include "camera.h"

/**
 * \file
 * \brief File IO functions for a \ref maptk::camera
 *
 * File format is the KRTD file.
 */

namespace maptk
{

/// Read in a KRTD file, producing a camera object
/**
 * \throws file_not_found_exception
 *    Thrown when the file could not be found on the file system.
 * \throws file_not_read_exception
 *    Thrown when the file could not be read or parsed for whatever reason.
 * \throws boost::filesystem::filesystem_error
 *    Boost exception thrown if something goes wrong with the underlying file
 *    read.
 *
 * \param file_path   The path to the file to read in.
 * \return A \c camera_d object representing the contents of the read-in file.
 */
camera_d
MAPTK_CORE_EXPORT
read_krtd_file(path_t const& file_path);


/// Output the given \c camera object to the specified file path
/**
 * If a file exists at the target location, it will be overwritten. If the
 * containing directory of the given path does not exist, it will be created
 * before the file is opened for writing.
 *
 * \throws file_write_exception
 *    Thrown when something prevents output of the file.
 * \throws boost::filesystem::filesystem:error
 *    Thrown when an underlying boost::filesystem call failes for system
 *    reasons.
 *
 * \param cam       The \c camera object to output.
 * \param file_path The path to output the file to.
 */
void
MAPTK_CORE_EXPORT
write_krtd_file(camera const& cam,
                path_t const& file_path);

}

#endif // MAPTK_CAMERA_IO_H_
