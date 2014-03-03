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
convert_image_default
::convert_image_default()
{

}

/// Copy Constructor
convert_image_default
::convert_image_default(const convert_image_default& other)
{

}

/// Default image converter ( does nothing )
image_container_sptr
convert_image_default
::convert(image_container_sptr img) const
{
  return img;
}

} // end namespace algo

} // end namespace maptk
