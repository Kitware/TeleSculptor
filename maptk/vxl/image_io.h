/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_VXL_IMAGE_IO_H_
#define MAPTK_VXL_IMAGE_IO_H_

#include "vxl_config.h"
#include <maptk/core/algo/image_io.h>

namespace maptk
{

namespace vxl
{

/// A class for using VXL to read and write images
class MAPTK_VXL_EXPORT image_io
  : public algo::algorithm_impl<image_io, algo::image_io>
{
public:
  /// Return the name of this implementation
  std::string impl_name() const { return "vxl"; }

  // No configuration yet for this class
  virtual void set_configuration(config_block_sptr /*config*/) { }
  virtual bool check_configuration(config_block_sptr /*config*/) const { return true; }

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

} // end namespace vxl

} // end namespace maptk


#endif // MAPTK_VXL_IMAGE_IO_H_
