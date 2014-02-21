/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_IMAGE_H_
#define MAPTK_IMAGE_H_

#include "core_config.h"

#include <cstddef>

#include <boost/shared_ptr.hpp>

namespace maptk
{

/// This class represents a block of image memory on the heap.
/**
 * The image object use shared pointers to this class.
 * Derived image memory classes can proved access to image memory
 * stored in other forms, such as on the GPU or in 3rd party data structures.
 */
class MAPTK_CORE_EXPORT image_memory
{
public:
  /// Default Constructor
  image_memory();

  /// Constructor - allocates n bytes
  /**
   * \param n bytes to allocate
   */
  image_memory(size_t n);

  /// Copy constructor
  /**
   * \param other The other image_memory to copy from.
   */
  image_memory(const image_memory& other);

  /// Assignment operator
  /**
   * Other image_memory whose internal data is copied into ours.
   * \param other image_memory to copy from.
   */
  image_memory& operator=(const image_memory& other);

  /// Destructor
  virtual ~image_memory();

  /// Return a pointer to the allocated memory
  virtual void* data();

  /// The number of bytes allocated
  size_t size() const { return size_; }

protected:
  /// The image data
  void *data_;

  /// The number of bytes allocated
  size_t size_;
};

/// Shared pointer for base image_memory type
typedef boost::shared_ptr<image_memory> image_memory_sptr;


/// The representation of an in-memory image.
/**
 * Images share memory using the image_memory class.
 */
class MAPTK_CORE_EXPORT image
{
public:
  /// Convenience typedef for the size of a byte
  typedef unsigned char byte;

  /// Default Constructor
  image();

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
  /**
   * The new image will share the same memory as the old image
   * \param other The other image.
   */
  image(const image& other);

  /// Assignment operator
  const image& operator=(const image& other);

  /// Const access to the image memory
  const image_memory_sptr& memory() const { return data_; }

  /// Access to the image memory
  image_memory_sptr memory() { return data_; }

  /// The size of the image data in bytes
  /**
   * This size includes all allocated image memory,
   * which could be larger than width*height*depth.
   */
  size_t size() const;

  /// Const access to the pointer to first image pixel
  /**
   * This may differ from \a data() if the image is a
   * window into a large image memory chunk.
   */
  const byte* first_pixel() const { return first_pixel_; }

  /// Access to the pointer to first image pixel
  /**
   * This may differ from \a data() if the image is a
   * window into a larger image memory chunk.
   */
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

  /// Access pixels in the first channel of the image
  /**
   * \param i width position (x)
   * \param j height position (y)
   */
  inline byte& operator()(unsigned i, unsigned j)
  {
    assert(i < width_);
    assert(j < height_);
    return first_pixel_[w_step_*i + h_step_*j];
  }

  /// Const access pixels in the first channel of the image
  inline const byte& operator()(unsigned i, unsigned j) const
  {
    assert(i < width_);
    assert(j < height_);
    return first_pixel_[w_step_*i + h_step_*j];
  }

  /// Access pixels in the image (width, height, channel)
  inline byte& operator()(unsigned i, unsigned j, unsigned k)
  {
    assert(i < width_);
    assert(j < height_);
    assert(k < depth_);
    return first_pixel_[w_step_*i + h_step_*j + d_step_*k];
  }

  /// Const access pixels in the image (width, height, channel)
  inline const byte& operator()(unsigned i, unsigned j, unsigned k) const
  {
    assert(i < width_);
    assert(j < height_);
    assert(k < depth_);
    return first_pixel_[w_step_*i + h_step_*j + d_step_*k];
  }

  /// Deep copy the image data from another image into this one
  void copy_from(const image& other);

  /// Set the size of the image.
  /**
   * If the size has not changed, do nothing.
   * Otherwise, allocate new memory matching the new size.
   * \param width a new image width
   * \param height a new image height
   * \param depth a new image depth
   */
  void set_size(size_t width, size_t height, size_t depth);

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


/// Compare to images to see if the pixels have the same values.
/**
 * This does not require that the images have the same memory layout,
 * only that the images have the same dimensions and pixel values.
 * \param img1 first image to compare
 * \param img2 second image to compare
 */
MAPTK_CORE_EXPORT bool equal_content(const image& img1, const image& img2);


} // end namespace maptk


#endif // MAPTK_IMAGE_H_
