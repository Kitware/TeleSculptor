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
camera_map_sptr
MAPTK_CORE_EXPORT
read_krtd_files(std::vector<path_t> const& img_files, path_t const& dir);


} // end namespace maptk


#endif // MAPTK_CAMERA_MAP_IO_H_
