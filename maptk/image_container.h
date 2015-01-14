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
 * \brief core image_container interface
 */

#ifndef MAPTK_IMAGE_CONTAINER_H_
#define MAPTK_IMAGE_CONTAINER_H_

#include <vector>
#include <boost/shared_ptr.hpp>
#include "image.h"


namespace maptk
{


/// An abstract representation of an image container.
/**
 * This class provides an interface for passing image data
 * between algorithms.  It is intended to be a wrapper for image
 * classes in third-party libraries and facilitate conversion between
 * various representations.  It provides limited access to the underlying
 * data and is not intended for direct use in image processing algorithms.
 */
class image_container
{
public:

  /// Destructor
  virtual ~image_container() {}

  /// The size of the image data in bytes
  /**
   * This size includes all allocated image memory,
   * which could be larger than width*height*depth.
   */
  virtual size_t size() const = 0;

  /// The width of the image in pixels
  virtual size_t width() const = 0;

  /// The height of the image in pixels
  virtual size_t height() const = 0;

  /// The depth (or number of channels) of the image
  virtual size_t depth() const = 0;

  /// Get an in-memory image class to access the data
  virtual image get_image() const = 0;

};


/// Shared pointer for base image_container type
typedef boost::shared_ptr<image_container> image_container_sptr;


/// List of image_container shared pointers
typedef std::vector<image_container_sptr> image_container_sptr_list;


/// This concrete image container is simply a wrapper around an image
class simple_image_container
: public image_container
{
public:

  /// Constructor
  explicit simple_image_container(const image& d)
  : data(d) {}

  /// The size of the image data in bytes
  /**
   * This size includes all allocated image memory,
   * which could be larger than width*height*depth.
   */
  virtual size_t size() const { return data.size(); }

  /// The width of the image in pixels
  virtual size_t width() const { return data.width(); }

  /// The height of the image in pixels
  virtual size_t height() const { return data.height(); }

  /// The depth (or number of channels) of the image
  virtual size_t depth() const { return data.depth(); }

  /// Get an in-memory image class to access the data
  virtual image get_image() const { return data; };

protected:
  /// data for this image container
  image data;
};


} // end namespace maptk


#endif // MAPTK_IMAGE_CONTAINER_H_
