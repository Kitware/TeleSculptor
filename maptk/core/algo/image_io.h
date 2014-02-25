/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief Interface for image_io \link maptk::algo::algorithm_def algorithm
 *        definition \endlink.
 */

#ifndef MAPTK_ALGO_IMAGE_IO_H_
#define MAPTK_ALGO_IMAGE_IO_H_

#include <string>

#include <boost/shared_ptr.hpp>

#include <maptk/core/algo/algorithm.h>
#include <maptk/core/image_container.h>

namespace maptk
{

namespace algo
{

/// An abstract base class for reading and writing images
class MAPTK_CORE_EXPORT image_io
  : public algorithm_def<image_io>
{
public:
  /// Return the name of this algorithm
  std::string type_name() const { return "image_io"; }

  /// Load image image from the file
  /**
   * \throws maptk::path_not_exists {
   *    Thrown when the given path does not exist.
   *    }
   * \throws maptk::path_not_a_file {
   *    Thrown when the given path does not point to a file (i.e. it points to
   *    a directory).
   *    }
   *
   * \param filename the path to the file the load
   * \returns an image container refering to the loaded image
   */
  image_container_sptr load(std::string const& filename) const;

  /// Save image image to a file
  /**
   * Image file format is based on file extension.
   *
   * \throws maptk::path_not_exists {
   *    Thrown when the expected containing directory of the given path does
   *    not exist.
   *    }
   * \throws maptk::path_not_a_directory {
   *    Thrown when the expected containing directory of the given path is
   *    not actually a directory.
   *    }
   *
   * \param filename the path to the file to save
   * \param data the image container refering to the image to write
   */
  void save(std::string const& filename, image_container_sptr data) const;

private:
  /// Implementation specific load functionality.
  /**
   * \param filename the path to the file the load
   * \returns an image container refering to the loaded image
   */
  virtual image_container_sptr load_(std::string const& filename) const = 0;
  /// Implementation specific save functionality.
  /**
   * \param filename the path to the file to save
   * \param data the image container refering to the image to write
   */
  virtual void save_(std::string const& filename,
                     image_container_sptr data) const = 0;
};


/// Shared pointer type for generic image_io definition type.
typedef boost::shared_ptr<image_io> image_io_sptr;


} // end namespace algo

} // end namespace maptk


#endif // MAPTK_ALGO_IMAGE_IO_H_
