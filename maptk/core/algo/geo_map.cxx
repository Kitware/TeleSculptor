/*ckwg +5
 * Copyright 2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include "geo_map.h"

namespace maptk
{

namespace algo
{

/// Return the standard zone number for a Latitude and Longitude
int
geo_map
::latlon_zone(double lat, double lon) const
{
  int ilon = static_cast<int>(lon);
  while(ilon < -180)
  {
    ilon += 360;
  }
  // this simplifed implementation ignores the exceptions to the
  // standard UTM zone rules (e.g. around Norway)
  return ((ilon + 186) / 6) % 60;
}

} // end namespace algo

} // end namespace maptk
