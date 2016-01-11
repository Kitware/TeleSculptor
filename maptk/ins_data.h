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
 * \brief core ins_data interface
 */

#ifndef MAPTK_INS_DATA_H_
#define MAPTK_INS_DATA_H_


#include <vital/vital_config.h>
#include <maptk/maptk_export.h>

#include <iostream>
#include <string>


namespace kwiver {
namespace maptk {

/// Inertial Navigation System (INS) data
/**
 * This struct hold IMU and GPS sensor readings
 * provided in the imagery metadata
 */
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
MAPTK_EXPORT bool operator ==(const ins_data& d1, const ins_data& d2);

/// inequality operator
MAPTK_EXPORT bool operator !=(const ins_data& d1, const ins_data& d2);

/// output stream operator for INS data
/**
 * \param s output stream
 * \param d ins_data to stream
 */
MAPTK_EXPORT std::ostream& operator<<(std::ostream& s, const ins_data& d);

/// input stream operator for a INS data
/**
 * \throws invalid_data When given stream contains data unsuitable for creating
 *                      an ins_data object.
 * \param s input stream
 * \param d ins_data to stream into
 */
MAPTK_EXPORT std::istream& operator>>(std::istream& s, ins_data& d);


} // end namespace maptk
} // end namespace kwiver


#endif // MAPTK_INS_DATA_H_
