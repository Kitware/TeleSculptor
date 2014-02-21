/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief geo_map algorithm general helper function
 */

#include "geo_map.h"
#include <maptk/core/algo/algorithm.txx>

/// \cond DoxygenSuppress
INSTANTIATE_ALGORITHM_DEF(maptk::algo::geo_map);
/// \endcond

namespace maptk
{

namespace algo
{

/// Return the standard zone number for a Latitude and Longitude
int
geo_map
::latlon_zone(double /*lat*/, double lon) const
{
  while(lon < -180)
  {
    lon += 360;
  }
  // this simplifed implementation ignores the exceptions to the
  // standard UTM zone rules (e.g. around Norway)
  return (static_cast<int>((lon + 180) / 6) % 60) + 1;
}

} // end namespace algo

} // end namespace maptk
