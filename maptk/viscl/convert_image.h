/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_VISCL_CONVERT_IMAGE_H_
#define MAPTK_VISCL_CONVERT_IMAGE_H_


#include <maptk/core/algo/convert_image.h>

#include "viscl_config.h"

namespace maptk
{

namespace vcl
{

/// Class to convert an image to a viscl base image
class MAPTK_VISCL_EXPORT convert_image
  : public algo::algorithm_impl<convert_image, algo::convert_image>
{
public:

  /// Default Constructor
  convert_image();

  /// Copy Constructor
  convert_image(const convert_image &);

  /// Return the name of this implementation
  std::string impl_name() const { return "viscl"; }

  /// Image convert to viscl underlying type
  /**
   * \param [in] img image to be converted
   * \returns the image container with underlying viscl img
   * should be used to prevent repeated image uploading to GPU
   */
  virtual image_container_sptr convert(image_container_sptr img) const;
};


} // end namespace vcl

} // end namespace maptk


#endif // MAPTK_VISCL_CONVERT_IMAGE_H_
