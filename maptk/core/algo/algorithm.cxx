/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include "algorithm.h"

namespace maptk
{

namespace algo
{


/// Get this alg's \link maptk::config_block configuration block \endlink
config_block_sptr
algorithm
::get_configuration()
{
  return config_block::empty_config(this->type_name());
}


} // end namespace algo

} // end namespace maptk
