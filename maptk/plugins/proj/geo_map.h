/*ckwg +29
 * Copyright 2013-2015 by Kitware, Inc.
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
 * \brief PROJ geo_map algorithm interface
 */

#ifndef MAPTK_PLUGINS_PROJ_GEO_MAP_H_
#define MAPTK_PLUGINS_PROJ_GEO_MAP_H_

#include <vital/algo/geo_map.h>
#include <maptk/plugins/proj/proj_config.h>


namespace maptk
{

namespace proj
{

/// PROJ implementation of geo_map algorithm
class MAPTK_PROJ_EXPORT geo_map
  : public kwiver::vital::algorithm_impl<geo_map, kwiver::vital::algo::geo_map>
{
public:

  /// Default Constructor
  geo_map() {}

  /// Return the name of this implementation
  virtual std::string impl_name() const { return "proj"; }

  // No configuration for this class yet
  /// \cond DoxygenSuppress
  virtual void set_configuration(kwiver::vital::config_block_sptr /*config*/) { }
  virtual bool check_configuration(kwiver::vital::config_block_sptr /*config*/) const { return true; }
  /// \endcond

  /// Convert UTM coordinate into latitude and longitude.
  /**
   * \param       easting     The easting (X) UTM coordinate in meters.
   * \param       northing    The northing (Y) UTM coordinate in meters.
   * \param       zone        The zone of the UTM coordinate (1-60).
   * \param       north_hemi  True if the UTM northing coordinate is in respect
   *                          to the northern hemisphere.
   * \param[out]  lat         Output latitude (Y) in decimal degrees.
   * \param[out]  lon         Output longiture (X) in decimal degrees.
   */
  virtual void utm_to_latlon(double easting, double northing,
                             int zone, bool north_hemi,
                             double& lat, double& lon) const;

  /// Convert latitude and longitude into UTM coordinates.
  /**
   * \param       lat         The latitude (Y) coordinate in decimal degrees.
   * \param       lon         The longitude (X) coordinate in decimal degrees.
   * \param[out]  easting     Output easting (X) coordinate in meters.
   * \param[out]  northing    Output northing (Y) coordinate in meters.
   * \param[out]  zone        Zone of the output UTM coordinate.
   * \param[out]  north_hemi  True if the output UTM coordinate northing is in
   *                          respect to the northern hemisphere. False if not.
   * \param       setzone     If a valid UTM zone, use the given zone instead
   *                          of the computed zone from the given lat/lon
   *                          coordinate.
   */
  virtual void latlon_to_utm(double lat, double lon,
                             double& easting, double& northing,
                             int& zone, bool& north_hemi,
                             int setzone=-1) const;

};// end class geo_map

} // end namespace proj

} // end namespace maptk

#endif // MAPTK_PLUGINS_PROJ_GEO_MAP_H_
