/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_PROJ_GEO_MAP_H_
#define MAPTK_PROJ_GEO_MAP_H_

#include <maptk/core/algo/geo_map.h>

namespace maptk
{

namespace proj
{

class proj_geo_map
: public  algo::algorithm_impl<proj_geo_map, algo::geo_map>
{
public:

  /// Default Constructor
  proj_geo_map() {}

  /// Return the name of this implementation
  std::string impl_name() const { return "proj"; }

  // No configuration for this class yet
  virtual void set_configuration(config_block_sptr /*config*/) { }
  virtual bool check_configuration(config_block_sptr /*config*/) const { return true; }

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

};// end class proj_geo_map

} // end namespace proj

} // end namespace maptk

#endif // MAPTK_PROJ_GEO_MAP_H_
