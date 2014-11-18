/*ckwg +29
 * Copyright 2013-2014 by Kitware, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither name of Kitware, Inc. nor the names of any contributors may be used
 *    to endorse or promote products derived from this software without specific
 *    prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHORS OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * \file
 * \brief PROJ geo_map algorithm implementation
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
/**
 * The UTM specification allows for the description of a point's northing to
 * be relative to the equator (is_northp) or the south pole (!is_northp). This
 * is the offset, in meters, between the two hemisphere notations.
 */
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

namespace proj
{

/// Convert UTM coordinate into latitude and longitude.
void
geo_map
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
  ret = 0;  // suppresses unused variable warning when build as release

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
geo_map
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
  ret = 0;  // suppresses unused variable warning when build as release

  // proj always returns with respect to the northern hemisphere.
  north_hemi = true;

  pj_free(pj_latlon);
  pj_free(pj_utm);
}

} // end namespace proj

} // end namespace maptk
