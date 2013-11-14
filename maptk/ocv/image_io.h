/*ckwg +5
 * Copyright 2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_OCV_IMAGE_IO_H_
#define MAPTK_OCV_IMAGE_IO_H_

#include <maptk/core/algo/image_io.h>

namespace maptk
{

namespace ocv
{

/// A class for using OpenCV to read and write images
class image_io
: public algo::image_io
{
public:
  /// Load image image from the file
  /// \param filename the path to the file the load
  /// \returns an image container refering to the loaded image
  virtual image_container_sptr load(const std::string& filename) const;

  /// Save image image to a file
  /// Image file format is based on file extension.
  /// \param filename the path to the file to save
  /// \param data the image container refering to the image to write
  virtual void save(const std::string& filename,
                    image_container_sptr data) const;
};

} // end namespace ocv

} // end namespace maptk


#endif // MAPTK_OCV_IMAGE_IO_H_
