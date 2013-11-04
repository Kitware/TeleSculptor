/*ckwg +5
 * Copyright 2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_IMAGE_H_
#define MAPTK_IMAGE_H_

#include <vector>
#include <boost/shared_ptr.hpp>
#include "feature.h"


namespace maptk
{

/// An abstract representation of an image.
///
/// This class provides an interface for passing image data
/// between algorithms.  It is intended to be a wrapper for image
/// classes in third-party libraries and facilitate conversion between
/// various representations.  It provides limited access to the underlying
/// data and is not intended for direct use in image processing algorithms.
/// \note for now only 8-bit byte images are supported
class image
{
public:
  /// Destructor
  virtual ~image() {}

  typedef unsigned char byte;

  /// Const access to the image data
  virtual const byte* data() const = 0;

  /// Access to the image data
  virtual byte* data() = 0;

  /// The size of the image data in bytes
  /// This size includes all allocated image memory,
  /// which could be larger than width*height*depth.
  virtual size_t size() const = 0;

  /// Const access to the pointer to first image pixel
  /// This may differ from \a data() if the image is a
  /// window into a larger image memory chunk.
  virtual const byte* first_pixel() const = 0;

  /// Access to the pointer to first image pixel
  /// This may differ from \a data() if the image is a
  /// window into a large image memory chunk.
  virtual byte* first_pixel() = 0;

  /// The width of the image in pixels
  virtual size_t width() const = 0;

  /// The height of the image in pixels
  virtual size_t height() const = 0;

  /// The depth (or number of channels) of the image
  virtual size_t depth() const = 0;

  /// The the step in memory to next pixel in the width direction
  virtual ptrdiff_t w_step() const = 0;

  /// The the step in memory to next pixel in the height direction
  virtual ptrdiff_t h_step() const = 0;

  /// The the step in memory to next pixel in the depth direction
  virtual ptrdiff_t d_step() const = 0;
};


/// The trivial concrete representation of an image.
class simple_image : public image
{
public:
  /// Default Constructor
  simple_image() {}

  /// Destructor
  virtual ~simple_image();

  /// Constructor that allocates image memory
  simple_image(size_t width, size_t height, size_t depth=1, bool interleave=false);

  /// Constructor that points at existing memory
  simple_image(const byte* first_pixel, size_t width, size_t height, size_t depth,
               ptrdiff_t w_step, ptrdiff_t h_step, ptrdiff_t d_step);

  /// Const access to the image data
  virtual const byte* data() const { return data_; }

  /// Access to the image data
  virtual byte* data() { return data_; }

  /// The size of the image data in bytes
  /// This size includes all allocated image memory,
  /// which could be larger than width*height*depth.
  virtual size_t size() const { return size_; }

  /// Const access to the pointer to first image pixel
  /// This may differ from \a data() if the image is a
  /// window into a large image memory chunk.
  virtual const byte* first_pixel() const { return first_pixel_; }

  /// Access to the pointer to first image pixel
  /// This may differ from \a data() if the image is a
  /// window into a larger image memory chunk.
  virtual byte* first_pixel() { return first_pixel_; }

  /// The width of the image in pixels
  virtual size_t width() const { return width_; }

  /// The height of the image in pixels
  virtual size_t height() const { return height_; }

  /// The depth (or number of channels) of the image
  virtual size_t depth() const { return depth_; }

  /// The the step in memory to next pixel in the width direction
  virtual ptrdiff_t w_step() const { return w_step_; }

  /// The the step in memory to next pixel in the height direction
  virtual ptrdiff_t h_step() const { return h_step_; }

  /// The the step in memory to next pixel in the depth direction
  virtual ptrdiff_t d_step() const { return d_step_; }

protected:

  /// The size of the data allocated in \a data_
  size_t size_;
  /// Pointer to memory owned by this class
  byte* data_;
  /// Pointer to the pixel at the origin
  byte* first_pixel_;
  /// Width of the image
  size_t width_;
  /// Height of the image
  size_t height_;
  /// Depth of the image (i.e. number of channels)
  size_t depth_;
  /// Increment to move to the next pixel along the width direction
  ptrdiff_t w_step_;
  /// Increment to move to the next pixel along the height direction
  ptrdiff_t h_step_;
  /// Increment to move to the next pixel along the depth direction
  ptrdiff_t d_step_;

};


} // end namespace maptk


#endif // MAPTK_IMAGE_H_
