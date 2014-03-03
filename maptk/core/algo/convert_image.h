/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_ALGO_CONVERT_IMAGE_H_
#define MAPTK_ALGO_CONVERT_IMAGE_H_

#include <string>

#include <boost/shared_ptr.hpp>

#include <maptk/core/algo/algorithm.h>
#include <maptk/core/image_container.h>

#include <maptk/core/core_config.h>

namespace maptk
{

namespace algo
{

/// An abstract base class for converting base image type
class MAPTK_CORE_EXPORT convert_image
  : public algorithm_def<convert_image>
{
public:
  /// Return the name of this algorithm
  std::string type_name() const { return "convert_image"; }

  /// Set this algorithm's properties via a config block
  virtual void set_configuration(config_block_sptr config);
  /// Check that the algorithm's currently configuration is valid
  virtual bool check_configuration(config_block_sptr config) const;

  /// Convert image base type
  virtual image_container_sptr convert(image_container_sptr img) const = 0;

};

typedef boost::shared_ptr<convert_image> convert_image_sptr;

/// A class for bypassing image conversion
class MAPTK_CORE_EXPORT convert_image_default
  : public algorithm_impl<convert_image_default, convert_image>
{
public:
   /// Default Constructor
  convert_image_default();

  /// Copy Constructor
  convert_image_default(const convert_image_default&);

  /// Return the name of this implementation
  std::string impl_name() const { return "default"; }

  /// Default image converter ( does nothing )
  /**
   * \param [in] img image to be converted
   * \returns the input image
   */
  virtual image_container_sptr convert(image_container_sptr img) const;
};


} // end namespace algo

} // end namespace maptk


#endif // MAPTK_ALGO_CONVERT_IMAGE_H_
