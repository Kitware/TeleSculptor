/*ckwg +29
 * Copyright 2013-2015 by Kitware, Inc.
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
 * \brief VXL image container interface
 */

#ifndef MAPTK_PLUGINS_VXL_IMAGE_CONTAINER_H_
#define MAPTK_PLUGINS_VXL_IMAGE_CONTAINER_H_


#include <vital/vital_config.h>
#include <maptk/plugins/vxl/maptk_vxl_export.h>

#include <vital/types/image_container.h>

#include <vil/vil_image_view.h>


namespace kwiver {
namespace maptk {

namespace vxl
{

/// This image container wraps a vil_image_view
/**
 * This class represents an image using vil_image_view format to store
 * the image data by extending the basic image_container.
 */
class MAPTK_VXL_EXPORT image_container
  : public vital::image_container
{
public:

  /// Constructor - from a vil_image_view
  explicit image_container(const vil_image_view<vxl_byte>& d)
  : data_(d) {}

  /// Constructor - convert maptk image to vil
  explicit image_container(const vital::image& maptk_image)
  : data_(maptk_to_vxl(maptk_image)) {}

  /// Constructor - convert base image container to vil
  explicit image_container(const vital::image_container& image_cont);

  /// Copy Constructor
  image_container(const maptk::vxl::image_container& other)
  : data_(other.data_) {}

  /// The size of the image data in bytes
  /**
   * This size includes all allocated image memory,
   * which could be larger than width*height*depth.
   */
  virtual size_t size() const;

  /// The width of the image in pixels
  virtual size_t width() const { return data_.ni(); }

  /// The height of the image in pixels
  virtual size_t height() const { return data_.nj(); }

  /// The depth (or number of channels) of the image
  virtual size_t depth() const { return data_.nplanes(); }

  /// Get an in-memory image class to access the data
  virtual vital::image get_image() const { return vxl_to_maptk(data_); }

  /// Get image data in this container.
  vil_image_view<vxl_byte> get_vil_image_view() const { return data_; }

  /// Convert a VXL vil_image_view to a MAPTK image
  static vital::image vxl_to_maptk(const vil_image_view<vxl_byte>& img);

  /// Convert a MAPTK image to a VXL vil_image_view
  static vil_image_view<vxl_byte> maptk_to_vxl(const vital::image& img);

protected:
  /// image data
  vil_image_view<vxl_byte> data_;
};


} // end namespace vxl

} // end namespace maptk
} // end namespace kwiver


#endif // MAPTK_PLUGINS_VXL_IMAGE_CONTAINER_H_
