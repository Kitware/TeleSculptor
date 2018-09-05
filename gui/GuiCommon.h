/*ckwg +29
 * Copyright 2018 by Kitware, Inc.
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
 *  * Neither the name Kitware, Inc. nor the names of any contributors may be
 *    used to endorse or promote products derived from this software without
 *    specific prior written permission.
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

#ifndef GUI_COMMON_H_
#define GUI_COMMON_H_

#include <qtStlUtil.h>

#include <vital/config/config_block_io.h>
#include <vital/types/image.h>
#include <vital/types/metadata_map.h>
#include <vital/vital_types.h>

#include <vtkImageData.h>
#include <vtkSmartPointer.h>

// Generates frame name basename in a standard format
// This version uses a map of all metadata
std::string frameName(kwiver::vital::frame_id_t frame,
                      kwiver::vital::metadata_map::map_metadata_t const& mdm);


// Generates frame name basename in a standard format
// This version uses a vector of metadata for the frame
std::string frameName(kwiver::vital::frame_id_t frame,
                      kwiver::vital::metadata_vector const& mdv);


// Generates frame name basename in a standard format
// This version uses a single metadata object for the frame
std::string frameName(kwiver::vital::frame_id_t frame,
                      kwiver::vital::metadata_sptr md);


// Loads config file from installed config location
kwiver::vital::config_block_sptr readConfig(std::string const& name);


// find the full path to the first matching file on the config search path
kwiver::vital::path_t findConfig(std::string const& name);


// converts a vital image to a vtkImage
// TODO:  move this method to a new implementation of image_container in a new
//        vtk arrow
vtkSmartPointer<vtkImageData> vitalToVtkImage(kwiver::vital::image& img);

#endif
