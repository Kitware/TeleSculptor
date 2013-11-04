/*ckwg +5
 * Copyright 2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_IMAGE_H_
#define MAPTK_IMAGE_H_

#include <cstddef>
#include <boost/shared_ptr.hpp>

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
  image_memory(size_t n);

  /// Copy constructor
  image_memory(const image_memory& other);

  /// Assignment operator
  image_memory& operator=(const image_memory& other);

  /// Destructor
  virtual ~image_memory();

  /// Return a pointer to the allocated memory
  virtual void* data();

  /// The number of bytes allocated
  size_t size() const { return size_; }

  /// Reallocate new memory of size n bytes
  /// If the size has not changed, this function does nothing.
  virtual void set_size(size_t n);

protected:
  /// The image data
  void *data_;

  /// THe number of bytes allocated
  size_t size_;
};

typedef boost::shared_ptr<image_memory> image_memory_sptr;


/// The representation of an in-memory image.
/// Images share memory using the image_memory class.
class image
{
public:
  typedef unsigned char byte;

  /// Default Constructor
  image() {}

  /// Constructor that allocates image memory
  image(size_t width, size_t height, size_t depth=1, bool interleave=false);

  /// Constructor that points at existing memory
  image(const byte* first_pixel, size_t width, size_t height, size_t depth,
        ptrdiff_t w_step, ptrdiff_t h_step, ptrdiff_t d_step);

  /// Constructor that shares memory with another image
  image(const image_memory_sptr& mem,
        const byte* first_pixel, size_t width, size_t height, size_t depth,
        ptrdiff_t w_step, ptrdiff_t h_step, ptrdiff_t d_step);

  /// Copy Constructor
  /// the new image will share the same memory as the old image
  image(const image& other);

  /// Assignment operator
  const image& operator=(const image& other);

  /// Const access to the image data
  const byte* data() const;

  /// Access to the image data
  byte* data();

  /// The size of the image data in bytes
  /// This size includes all allocated image memory,
  /// which could be larger than width*height*depth.
  size_t size() const;

  /// Const access to the pointer to first image pixel
  /// This may differ from \a data() if the image is a
  /// window into a large image memory chunk.
  const byte* first_pixel() const { return first_pixel_; }

  /// Access to the pointer to first image pixel
  /// This may differ from \a data() if the image is a
  /// window into a larger image memory chunk.
  byte* first_pixel() { return first_pixel_; }

  /// The width of the image in pixels
  size_t width() const { return width_; }

  /// The height of the image in pixels
  size_t height() const { return height_; }

  /// The depth (or number of channels) of the image
  size_t depth() const { return depth_; }

  /// The the step in memory to next pixel in the width direction
  ptrdiff_t w_step() const { return w_step_; }

  /// The the step in memory to next pixel in the height direction
  ptrdiff_t h_step() const { return h_step_; }

  /// The the step in memory to next pixel in the depth direction
  ptrdiff_t d_step() const { return d_step_; }

protected:

  /// Smart pointer to memory viewed by this class
  image_memory_sptr data_;
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
