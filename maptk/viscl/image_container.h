/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_VISCL_IMAGE_CONTAINER_H_
#define MAPTK_VISCL_IMAGE_CONTAINER_H_

#include <maptk/core/image_container.h>


namespace maptk
{

namespace viscl
{


/// This image container wraps a VisCL image
class viscl_image_container
: public image_container
{
public:

  /// Constructor - from a VisCL image
  // TODO define 'type'
  //explicit viscl_image_container(const type& d)
  //: data_(d) {}

  /// Constructor - convert maptk image to VisCL image
  // TODO define 'type'
  //explicit viscl_image_container(const image& maptk_image)
  //: data_(maptk_to_viscl(maptk_image)) {}

  /// Constructor - convert base image container to VisCL
  explicit viscl_image_container(const image_container& image_cont);

  /// Copy Constructor
  // TODO define data_
  //viscl_image_container(const viscl_image_container& other)
  //: data_(other.data_) {}

  /// The size of the image data in bytes
  /// This size includes all allocated image memory,
  /// which could be larger than width*height*depth.
  virtual size_t size() const;

  /// The width of the image in pixels
  virtual size_t width() const { return 0; } // TODO implement

  /// The height of the image in pixels
  virtual size_t height() const { return 0; } // TODO implement

  /// The depth (or number of channels) of the image
  virtual size_t depth() const { return 0; } // TODO implement

  /// Get and in-memory image class to access the data
  virtual image get_image() const { return viscl_to_maptk(data_); }

  /// Access the underlying VisCL data structure
  // TODO implement
  //type get_viscl_image() const { return data_; }

  /// Convert a VisCL image to a MAPTK image
  // TODO implement
  //static image viscl_to_maptk(const type& img);

  /// Convert a MAPTK image to a VisCL image
  // TODO implement
  //static type maptk_to_viscl(const image& img);

protected:

  // TODO set VisCL image data type
  //type data_;
};


/// Extract a VisCL image from any image container
/**
 * If \a img is actually a viscl_image_container then
 * return the underlying VisCL image.  Otherwise, convert the image data
 * and upload to the GPU.
 */
// TODO implement
//type image_container_to_viscl(const image_container& img);


} // end namespace viscl

} // end namespace maptk


#endif // MAPTK_VISCL_IMAGE_CONTAINER_H_
