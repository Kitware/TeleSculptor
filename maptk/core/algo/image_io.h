/*ckwg +5
 * Copyright 2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_ALGO_IMAGE_IO_H_
#define MAPTK_ALGO_IMAGE_IO_H_

#include <string>
#include <maptk/core/algo/algorithm.h>
#include <maptk/core/image_container.h>
#include <boost/shared_ptr.hpp>

namespace maptk
{

namespace algo
{

/// An abstract base class for reading and writing images
class image_io : public algorithm_def<image_io>
{
public:
  /// Return the name of this algorithm
  std::string type_name() const { return "image_io"; }

  /// Load image image from the file
  /// \param filename the path to the file the load
  /// \returns an image container refering to the loaded image
  virtual image_container_sptr load(const std::string& filename) const = 0;

  /// Save image image to a file
  /// Image file format is based on file extension.
  /// \param filename the path to the file to save
  /// \param data the image container refering to the image to write
  virtual void save(const std::string& filename,
                    image_container_sptr data) const = 0;
};


typedef boost::shared_ptr<image_io> image_io_sptr;


} // end namespace algo

} // end namespace maptk


#endif // MAPTK_ALGO_IMAGE_IO_H_
