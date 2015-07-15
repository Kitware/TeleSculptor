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
 * \brief Interface for image_io \link maptk::algo::algorithm_def algorithm
 *        definition \endlink.
 */

#ifndef MAPTK_ALGO_IMAGE_IO_H_
#define MAPTK_ALGO_IMAGE_IO_H_

#include <maptk/config.h>

#include <string>

#include <boost/shared_ptr.hpp>

#include <vital/algo/algorithm.h>
#include <vital/types/image_container.h>

namespace maptk
{

namespace algo
{

/// An abstract base class for reading and writing images
/**
 * This class represents an abstract interface for reading and writing
 * images.
 */
class MAPTK_LIB_EXPORT image_io
  : public kwiver::vital::algorithm_def<image_io>
{
public:
  virtual ~image_io() MAPTK_DEFAULT_DTOR;

  /// Return the name of this algorithm
  static std::string static_type_name() { return "image_io"; }

  /// Load image image from the file
  /**
   * \throws kwiver::vital::path_not_exists Thrown when the given path does not exist.
   *
   * \throws kwiver::vital::path_not_a_file Thrown when the given path does
   *    not point to a file (i.e. it points to a directory).
   *
   * \param filename the path to the file the load
   * \returns an image container refering to the loaded image
   */
  kwiver::vital::image_container_sptr load(std::string const& filename) const;

  /// Save image image to a file
  /**
   * Image file format is based on file extension.
   *
   * \throws kwiver::vital::path_not_exists Thrown when the expected
   *    containing directory of the given path does not exist.
   *
   * \throws kwiver::vital::path_not_a_directory Thrown when the expected
   *    containing directory of the given path is not actually a
   *    directory.
   *
   * \param filename the path to the file to save
   * \param data the image container refering to the image to write
   */
  void save(std::string const& filename, kwiver::vital::image_container_sptr data) const;

private:
  /// Implementation specific load functionality.
  /**
   * Concrete implementations of image_io class must provide an
   * implementation for this method.
   *
   * \param filename the path to the file the load
   * \returns an image container refering to the loaded image
   */
  virtual kwiver::vital::image_container_sptr load_(std::string const& filename) const = 0;

  /// Implementation specific save functionality.
  /**
   * Concrete implementations of image_io class must provide an
   * implementation for this method.
   *
   * \param filename the path to the file to save
   * \param data the image container refering to the image to write
   */
  virtual void save_(std::string const& filename,
                     kwiver::vital::image_container_sptr data) const = 0;
};


/// Shared pointer type for generic image_io definition type.
typedef boost::shared_ptr<image_io> image_io_sptr;


} // end namespace algo

} // end namespace maptk


#endif // MAPTK_ALGO_IMAGE_IO_H_
