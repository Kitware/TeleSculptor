/*ckwg +5
 * Copyright 2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include "geo_map.h"

#include <assert.h>
#include <iostream>
#include <sstream>

#include <proj_api.h>

using namespace std;

// Private Helper functions
namespace
{

/// Meters aligning north and south halves of a UTM zone (10^7)
static const int utm_shift = 10000000;

/// Create and return a UTM PROJ projection in WGS84
/**
 * Projection will have reference to the given zone.
 * \param   zone  Zone to assign to the UTM projection object.
 * \return        The generated projection object, or NULL if we failed to
 *                create the projection.
 */
static projPJ gen_utm_pj(int zone)
{
  stringstream utm_config;
  utm_config << "+proj=utm +ellps=WGS84 +zone=" << zone;
  return pj_init_plus(utm_config.str().c_str());
}

/// Create and return a lonlat PROJ projection in WGS84
/**
 * \return The generated projection object, or NULL if we failed to create
 *         the projection.
 */
static projPJ gen_latlon_pj()
{
  return pj_init_plus("+proj=lonlat +ellps=WGS84");
}

}

namespace maptk
{

namespace algo
{

/// Convert UTM coordinate into latitude and longitude.
void
proj_geo_map
::utm_to_latlon(double easting, double northing,
                int zone, bool north_hemi,
                double& lat, double& lon) const
{
  // Zone should be valid of course
  assert(zone >= 1 && zone <= 60);

  // Create PROJ.4 transformation objects
  projPJ pj_latlon = gen_latlon_pj(),
         pj_utm = gen_utm_pj(zone);

  // force northing to be relative to equator
  if(! north_hemi)
    northing -= utm_shift;

  lon = easting;
  lat = northing;
  int ret = pj_transform(pj_utm, pj_latlon, 1, 1,
                         &lon, &lat, NULL);
  assert(ret == 0);

  // lat/lon now in radians, so need to convert to degrees
  lat /= DEG_TO_RAD;
  lon /= DEG_TO_RAD;

  // TODO: If we find out that this freeing takes a non-trivial amount of time
  // and we have extra RAM to work with, we could make a JIT cache of UTM
  // projections.
  pj_free(pj_latlon);
  pj_free(pj_utm);
}

/// Convert latitude and longitude into UTM coordinates.
void
proj_geo_map
::latlon_to_utm(double lat, double lon,
                double& easting, double& northing,
                int& zone, bool& north_hemi,
                int setzone) const
{
  // If we're given a valid zone via setzone, use that. Else introspect from
  // the given latlon values.
  if(setzone >= 1 && setzone <= 60)
    zone = setzone;
  else
    zone = this->latlon_zone(lat, lon);

  projPJ pj_latlon = gen_latlon_pj(),
         pj_utm = gen_utm_pj(zone);

  // PROJ wants radians
  easting = lon * DEG_TO_RAD;
  northing = lat * DEG_TO_RAD;
  int ret = pj_transform(pj_latlon, pj_utm, 1, 1,
                         &easting, &northing, NULL);
  assert(ret == 0);

  // proj always returns with respect to the northern hemisphere.
  north_hemi = true;

  pj_free(pj_latlon);
  pj_free(pj_utm);
}

} // end namespace algo

} // end namespace maptk
