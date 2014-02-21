/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief core ins_data interface
 */

#ifndef MAPTK_INS_DATA_H_
#define MAPTK_INS_DATA_H_

#include "core_config.h"

#include <iostream>
#include <string>

namespace maptk
{

/// Inertial Navigation System (INS) data
///
/// This struct hold IMU and GPS sensor readings
/// provided in the imagery metadata
struct ins_data
{
public:
  /// Default Constructor
  ins_data()
  : source_name("MAPTK"),
    yaw(0.0), pitch(0.0), roll(0.0),
    lat(0.0), lon(0.0),   alt(0.0),
    gps_sec(0.0), gps_week(0),
    n_vel(-1.0), e_vel(-1.0), up_vel(-1.0),
    imu_status(-1), local_adj(0), dst_flag(0)
  {}

  /// Constructor - from INS data
  ins_data(double y,  double p,  double r,
           double lt, double ln, double a,
           const std::string& name = "MAPTK",
           double gs=0.0, int gw=0,
           double nv=-1.0, double ev=-1.0, double uv=-1.0,
           int is=-1, int la=0, int df=0)
  : source_name(name),
    yaw(y),  pitch(p), roll(r),
    lat(lt), lon(ln),  alt(a),
    gps_sec(gs), gps_week(gw),
    n_vel(nv), e_vel(ev), up_vel(uv),
    imu_status(is), local_adj(la), dst_flag(df)
  {}

  /// Name of the source producing the data
  std::string source_name;
  /// Sensor yaw angle
  double yaw;
  /// Sensor pitch angle
  double pitch;
  /// Sensor roll angle
  double roll;
  /// Sensor latitude
  double lat;
  /// Sensor longitude
  double lon;
  /// Sensor altitude
  double alt;
  /// GPS time in seconds
  double gps_sec;
  /// GPS time - week of year
  int gps_week;
  /// Velocity in the North direction
  double n_vel;
  /// Velocity in the East direction
  double e_vel;
  /// Velocity in the up direction
  double up_vel;
  /// IMU status
  int imu_status;
  /// Local Adjustment
  int local_adj;
  /// Flags ?
  int dst_flag;
};

/// equality operator
MAPTK_CORE_EXPORT bool operator ==(const ins_data& d1, const ins_data& d2);

/// inequality operator
MAPTK_CORE_EXPORT bool operator !=(const ins_data& d1, const ins_data& d2);

/// output stream operator for INS data
/**
 * \param s output stream
 * \param d ins_data to stream
 */
MAPTK_CORE_EXPORT std::ostream& operator<<(std::ostream& s, const ins_data& d);

/// input stream operator for a INS data
/**
 * \param s input stream
 * \param d ins_data to stream into
 */
MAPTK_CORE_EXPORT std::istream& operator>>(std::istream& s, ins_data& d);


} // end namespace maptk


#endif // MAPTK_INS_DATA_H_
