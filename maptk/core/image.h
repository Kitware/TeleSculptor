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

/// This class represents a block of image memory on the heap.
/// The image object use shared pointers to this class.
/// Derived image memory classes can proved access to image memory
/// stored in other forms, such as on the GPU or in 3rd party data structures.
class image_memory
{
public:
  /// Default Constructor
  image_memory();

  /// Constructor - allocates n bytes
  image_memory(std::size_t n);

  /// Copy constructor
  image_memory(const image_memory& other);

  /// Assignment operator
  image_memory& operator=(const image_memory& other);

  /// Destructor
  virtual ~image_memory();

  /// Return a pointer to the allocated memory
  virtual void* data();

  /// The number of bytes allocated
  std::size_t size() const { return size_; }

  /// Reallocate new memory of size n bytes
  /// If the size has not changed, this function does nothing.
  virtual void set_size(std::size_t n);

protected:
  /// The image data
  void *data_;

  /// THe number of bytes allocated
  std::size_t size_;
};


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
  typedef boost::shared_ptr<image_memory> memory_sptr;

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
  virtual const byte* data() const;

  /// Access to the image data
  virtual byte* data();

  /// The size of the image data in bytes
  /// This size includes all allocated image memory,
  /// which could be larger than width*height*depth.
  virtual size_t size() const;

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

  /// Smart pointer to memory viewed by this class
  memory_sptr data_;
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
