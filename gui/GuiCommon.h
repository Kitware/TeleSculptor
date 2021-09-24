// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

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
                      kwiver::vital::metadata_map const& mdm);

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
vtkSmartPointer<vtkImageData> vitalToVtkImage(kwiver::vital::image const& img);

#endif
