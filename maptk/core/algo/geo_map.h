/*ckwg +5
 * Copyright 2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_ALGO_GEO_MAP_H_
#define MAPTK_ALGO_GEO_MAP_H_


namespace maptk
{

namespace algo
{


/// A base class for geographic conversions
class geo_map
{
public:
  /// Default Constructor
  geo_map() {}

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
                             double& lat, double& lon) const = 0;

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
                             int setzone=-1) const = 0;

  /// Return the standard zone number for a given latitude and longitude
  /**
   * \param lat latitude in decimal degrees.
   * \param lon longitude in decimal degrees.
   */
  virtual int latlon_zone(double lat, double lon) const;

};


} // end namespace algo

} // end namespace maptk


#endif // MAPTK_ALGO_GEO_MAP_H_
