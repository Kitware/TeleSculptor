/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include <maptk/core/algo/algorithm.txx>
#include "convert_image.h"

INSTANTIATE_ALGORITHM_DEF(maptk::algo::convert_image);

namespace maptk
{

namespace algo
{

/// Set this algorithm's properties via a config block
void
convert_image
::set_configuration(config_block_sptr config)
{

}

/// Check that the algorithm's current configuration is valid
bool
convert_image
::check_configuration(config_block_sptr config) const
{
  return true;
}

/// Default Constructor
default_convert_image
::default_convert_image()
{

}

/// Copy Constructor
default_convert_image
::default_convert_image(const default_convert_image& other)
{

}

/// Default image converter ( does nothing )
image_container_sptr
default_convert_image
::convert(image_container_sptr img) const
{
  return img;
}

} // end namespace algo

} // end namespace maptk
