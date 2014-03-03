/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include "convert_image.h"
#include "image_container.h"

namespace maptk
{

namespace vcl
{

/// Default Constructor
convert_image
::convert_image()
{

}

/// Copy Constructor
convert_image
::convert_image(const convert_image &)
{

}

/// Image convert to viscl underlying type
image_container_sptr
convert_image
::convert(image_container_sptr img) const
{
  return boost::shared_ptr<image_container>(new viscl_image_container(*img));
}

} // end namespace vcl

} // end namespace maptk
