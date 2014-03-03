/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_VISCL_IMAGE_CONTAINER_H_
#define MAPTK_VISCL_IMAGE_CONTAINER_H_

#include <maptk/core/image_container.h>
#include <maptk/viscl/viscl_config.h>
#include <viscl/core/image.h>

namespace maptk
{

namespace vcl
{

/// This image container wraps a VisCL image
class MAPTK_VISCL_EXPORT viscl_image_container
: public image_container
{
public:

  /// Constructor - from a VisCL image
  explicit viscl_image_container(const viscl::image& d)
  : data_(d) {}

  /// Constructor - convert maptk image to VisCL image
  explicit viscl_image_container(const image& maptk_image)
  : data_(maptk_to_viscl(maptk_image)) {}

  /// Constructor - convert base image container to VisCL
  explicit viscl_image_container(const image_container& image_cont);

  /// Copy Constructor
  viscl_image_container(const viscl_image_container& other)
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
  virtual image get_image() const { return viscl_to_maptk(data_); }

  /// Access the underlying VisCL data structure
  viscl::image get_viscl_image() const { return data_; }

  /// Convert a VisCL image to a MAPTK image
  static image viscl_to_maptk(const viscl::image& img);

  /// Convert a MAPTK image to a VisCL image
  static viscl::image maptk_to_viscl(const image& img);

protected:

  viscl::image data_;
};


/// Extract a VisCL image from any image container
/**
 * If \a img is actually a viscl_image_container then
 * return the underlying VisCL image.  Otherwise, convert the image data
 * and upload to the GPU.
 */
MAPTK_VISCL_EXPORT viscl::image image_container_to_viscl(const image_container& img);


} // end namespace vcl

} // end namespace maptk


#endif // MAPTK_VISCL_IMAGE_CONTAINER_H_
