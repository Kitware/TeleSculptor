/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_IMAGE_CONTAINER_H_
#define MAPTK_IMAGE_CONTAINER_H_

#include <vector>
#include <boost/shared_ptr.hpp>
#include "image.h"


namespace maptk
{

/// An abstract representation of an image container.
///
/// This class provides an interface for passing image data
/// between algorithms.  It is intended to be a wrapper for image
/// classes in third-party libraries and facilitate conversion between
/// various representations.  It provides limited access to the underlying
/// data and is not intended for direct use in image processing algorithms.
class image_container
{
public:

  /// Destructor
  virtual ~image_container() {}

  /// The size of the image data in bytes
  /// This size includes all allocated image memory,
  /// which could be larger than width*height*depth.
  virtual size_t size() const = 0;

  /// The width of the image in pixels
  virtual size_t width() const = 0;

  /// The height of the image in pixels
  virtual size_t height() const = 0;

  /// The depth (or number of channels) of the image
  virtual size_t depth() const = 0;

  /// Get and in-memory image class to access the data
  virtual image get_image() const = 0;

};

/// Shared pointer for base image_container type
typedef boost::shared_ptr<image_container> image_container_sptr;


/// This concrete image container is simply a wrapper around an image
class simple_image_container
: public image_container
{
public:

  /// Constructor
  explicit simple_image_container(const image& d)
  : data(d) {}

  /// The size of the image data in bytes
  /// This size includes all allocated image memory,
  /// which could be larger than width*height*depth.
  virtual size_t size() const { return data.size(); }

  /// The width of the image in pixels
  virtual size_t width() const { return data.width(); }

  /// The height of the image in pixels
  virtual size_t height() const { return data.height(); }

  /// The depth (or number of channels) of the image
  virtual size_t depth() const { return data.depth(); }

  /// Get and in-memory image class to access the data
  virtual image get_image() const { return data; };

protected:
  /// data for this image container
  image data;
};


} // end namespace maptk


#endif // MAPTK_IMAGE_CONTAINER_H_
