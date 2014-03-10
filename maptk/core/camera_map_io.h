/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief Header file for loading camera maps
 */

#ifndef MAPTK_CAMERA_MAP_IO_H_
#define MAPTK_CAMERA_MAP_IO_H_

#include "types.h"
#include "camera_map.h"

#include <vector>

namespace maptk
{


/// Load a camera map from krtd files stored in a directory.
/**
 * This function assumes that krtd files stored in the directory have the
 * same names as those in an image file list, only with a .krtd extension
 * instead of an image extension.
 *
 * \throws invalid_data
 *   Unable to find any camera krtd files in the given directory
 * \throw path_not_exists
 *   The specified directory does not exist
 *
 * \param img_files a list of image files
 * \param dir directory path containing krtd files for the given images
 * \return a new camera map created after parsing all krtd files
 */
camera_map_sptr
MAPTK_CORE_EXPORT
read_krtd_files(std::vector<path_t> const& img_files, path_t const& dir);


} // end namespace maptk


#endif // MAPTK_CAMERA_MAP_IO_H_
