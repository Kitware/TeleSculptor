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
  geo_map();

  /// Convert UTM coordinates to Latitude and Longitude
  /// \param east is the easting (X) UTM coordnate
  /// \param north is the northing (Y) UTM coordinate
  /// \param zone is the UTM zone (values 1-60)
  /// \param north_hemi is true if in the northern hemisphere, false for southern
  /// \param lat is the output latitude in degrees
  /// \param lon is the output longitude in degrees
  virtual void utm_to_latlon(double east, double north,
                             int zone, bool north_hemi,
                             double& lat, double& lon) const = 0;

  /// Convert Latitude and Longitude coordinates to UTM
  /// \param lat is the output latitude in degrees
  /// \param lon is the output longitude in degrees
  /// \param zone is the UTM zone (values 1-60)
  /// \param north_hemi is true if in the northern hemisphere, false for southern
  /// \param east is the easting (X) UTM coordnate
  /// \param north is the northing (Y) UTM coordinate
  /// \param setzone if a valid zone, use this instead of the computed one.
  virtual void latlon_to_utm(double lat, double lon,
                             double& east, double& north,
                             int& zone, bool& north_hemi,
                             int setzone=-1) const = 0;

  /// Return the standard zone number for a Latitude and Longitude
  /// \param lat is the output latitude in degrees
  /// \param lon is the output longitude in degrees
  /// \returns the standard UTM zone number for this location.
  virtual int latlon_zone(double lat, double lon) const;

};


} // end namespace algo

} // end namespace maptk


#endif // MAPTK_ALGO_GEO_MAP_H_
