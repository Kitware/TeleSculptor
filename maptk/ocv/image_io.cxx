/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief OpenCV image_io implementation
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
/**
 * \param filename the path to the file the load
 * \returns an image container refering to the loaded image
 */
image_container_sptr
image_io
::load_(const std::string& filename) const
{
  cv::Mat img = cv::imread(filename.c_str());
  return image_container_sptr(new ocv::image_container(img));
}


/// Save image image to a file
/**
 * \param filename the path to the file to save.
 * \param data The image container refering to the image to write.
 */
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
