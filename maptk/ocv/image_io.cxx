/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include "image_io.h"
#include "image_container.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace maptk
{

namespace ocv
{

/// Load image image from the file
image_container_sptr
image_io
::load_(const std::string& filename) const
{
  cv::Mat img = cv::imread(filename.c_str());
  return image_container_sptr(new ocv::image_container(img));
}


/// Save image image to a file
void
image_io
::save_(const std::string& filename,
       image_container_sptr data) const
{
  cv::imwrite(filename.c_str(),
              ocv::image_container::maptk_to_ocv(data->get_image()));
}

} // end namespace ocv

} // end namespace maptk
