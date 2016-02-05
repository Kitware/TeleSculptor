/*ckwg +29
 * Copyright 2014-2016 by Kitware, Inc.
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

#ifndef MAPTK_PLUGINS_VISCL_IMAGE_CONTAINER_H_
#define MAPTK_PLUGINS_VISCL_IMAGE_CONTAINER_H_


#include <vital/vital_config.h>
#include <maptk/plugins/viscl/maptk_viscl_export.h>

#include <vital/types/image_container.h>

#include <viscl/core/image.h>


namespace kwiver {
namespace maptk {

namespace vcl
{

/// This image container wraps a VisCL image
class MAPTK_VISCL_EXPORT image_container
: public vital::image_container
{
public:

  /// Constructor - from a VisCL image
  explicit image_container(const viscl::image& d)
  : data_(d) {}

  /// Constructor - convert maptk image to VisCL image
  explicit image_container(const vital::image& maptk_image)
  : data_(maptk_to_viscl(maptk_image)) {}

  /// Constructor - convert base image container to VisCL
  explicit image_container(const vital::image_container& image_cont);

  /// Copy Constructor
  image_container(const maptk::vcl::image_container& other)
  : data_(other.data_) {}

  /// The size of the image data in bytes
  /**
    * This size includes all allocated image memory,
    * which could be larger than width*height*depth.
    */
  virtual size_t size() const;

  /// The width of the image in pixels
  virtual size_t width() const { return data_.width(); }

  /// The height of the image in pixels
  virtual size_t height() const { return data_.height(); }

  /// The depth (or number of channels) of the image
  /**
    * viscl images only support 1 plane images at the moment
    */
  virtual size_t depth() const { return data_.depth(); }

  /// Get and in-memory image class to access the data
  virtual vital::image get_image() const { return viscl_to_maptk(data_); }

  /// Access the underlying VisCL data structure
  viscl::image get_viscl_image() const { return data_; }

  /// Convert a VisCL image to a MAPTK image
  static vital::image viscl_to_maptk(const viscl::image& img);

  /// Convert a MAPTK image to a VisCL image
  static viscl::image maptk_to_viscl(const vital::image& img);

protected:

  viscl::image data_;
};


/// Extract a VisCL image from any image container
/**
 * If \a img is actually a vcl::image_container then
 * return the underlying VisCL image.  Otherwise, convert the image data
 * and upload to the GPU.
 */
MAPTK_VISCL_EXPORT viscl::image image_container_to_viscl(const vital::image_container& img);


} // end namespace vcl

} // end namespace maptk
} // end namespace kwiver


#endif // MAPTK_PLUGINS_VISCL_IMAGE_CONTAINER_H_
