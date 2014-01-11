/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include <maptk/proj/register.h>
#include <maptk/proj/geo_map.h>

namespace maptk
{

namespace proj
{

/// register all algorithms in this module
void register_algorithms()
{
  proj::geo_map::register_self();
}


} // end namespace proj

} // end namespace maptk
