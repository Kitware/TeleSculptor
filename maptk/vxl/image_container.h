/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_VXL_IMAGE_CONTAINER_H_
#define MAPTK_VXL_IMAGE_CONTAINER_H_

#include "vxl_config.h"

#include <vil/vil_image_view.h>

#include <maptk/core/image_container.h>


namespace maptk
{

namespace vxl
{


/// This image container wraps a vil_image_view
class MAPTK_VXL_EXPORT image_container
  : public maptk::image_container
{
public:

  /// Constructor - from a vil_image_view
  explicit image_container(const vil_image_view<vxl_byte>& d)
  : data_(d) {}

  /// Constructor - convert maptk image to vil
  explicit image_container(const image& maptk_image)
  : data_(maptk_to_vxl(maptk_image)) {}

  /// Constructor - convert base image container to vil
  explicit image_container(const maptk::image_container& image_cont);

  /// Copy Constructor
  image_container(const maptk::vxl::image_container& other)
  : data_(other.data_) {}

  /// The size of the image data in bytes
  /// This size includes all allocated image memory,
  /// which could be larger than width*height*depth.
  virtual size_t size() const;

  /// The width of the image in pixels
  virtual size_t width() const { return data_.ni(); }

  /// The height of the image in pixels
  virtual size_t height() const { return data_.nj(); }

  /// The depth (or number of channels) of the image
  virtual size_t depth() const { return data_.nplanes(); }

  /// Get and in-memory image class to access the data
  virtual image get_image() const { return vxl_to_maptk(data_); }

  vil_image_view<vxl_byte> get_vil_image_view() const { return data_; }

  /// Convert a VXL vil_image_view to a MAPTK image
  static image vxl_to_maptk(const vil_image_view<vxl_byte>& img);

  /// Convert a MAPTK image to a VXL vil_image_view
  static vil_image_view<vxl_byte> maptk_to_vxl(const image& img);

protected:

  vil_image_view<vxl_byte> data_;
};


} // end namespace vxl

} // end namespace maptk


#endif // MAPTK_IMAGE_CONTAINER_H_
