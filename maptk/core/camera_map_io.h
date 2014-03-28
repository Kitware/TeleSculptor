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
