/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief File IO functions for a \ref maptk::ins_data
 *
 * File format is the POS file.
 */

#ifndef MAPTK_INS_DATA_IO_H_
#define MAPTK_INS_DATA_IO_H_

#include "core_config.h"
#include "types.h"

#include "ins_data.h"


namespace maptk
{

/// Read in a POS file, producing an ins_data object
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
 * \return A \c ins_data object representing the contents of the read-in file.
 */
ins_data
MAPTK_CORE_EXPORT
read_pos_file(path_t const& file_path);

/// Output the given \c ins_data object to the specified file path
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
 * \param ins       The \c ins_data object to output.
 * \param file_path The path to output the file to.
 */
void
MAPTK_CORE_EXPORT
write_pos_file(ins_data const& ins,
               path_t const& file_path);

}

#endif // MAPTK_INS_DATA_IO_H_
