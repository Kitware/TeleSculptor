/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include "image_io.h"
#include "image_container.h"
#include <vil/vil_load.h>
#include <vil/vil_save.h>

namespace maptk
{

namespace vxl
{

/// Load image image from the file
image_container_sptr
image_io
::load(const std::string& filename) const
{
  vil_image_view<vxl_byte> img = vil_load(filename.c_str());
  return image_container_sptr(new vxl::image_container(img));
}


/// Save image image to a file
void
image_io
::save(const std::string& filename,
       image_container_sptr data) const
{
  vil_save(vxl::image_container::maptk_to_vxl(data->get_image()),
           filename.c_str());
}

} // end namespace vxl

} // end namespace maptk
