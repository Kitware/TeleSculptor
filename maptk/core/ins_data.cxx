/*ckwg +5
 * Copyright 2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include "ins_data.h"

#include <vector>
#include <sstream>


namespace maptk
{

/// equality operator
bool operator ==(const ins_data& d1, const ins_data& d2)
{
  return d1.source_name == d2.source_name &&
         d1.yaw         == d2.yaw         &&
         d1.pitch       == d2.pitch       &&
         d1.roll        == d2.roll        &&
         d1.lat         == d2.lat         &&
         d1.lon         == d2.lon         &&
         d1.alt         == d2.alt         &&
         d1.gps_sec     == d2.gps_sec     &&
         d1.gps_week    == d2.gps_week    &&
         d1.n_vel       == d2.n_vel       &&
         d1.e_vel       == d2.e_vel       &&
         d1.up_vel      == d2.up_vel      &&
         d1.imu_status  == d2.imu_status  &&
         d1.local_adj   == d2.local_adj   &&
         d1.dst_flag    == d2.dst_flag    ;
}

/// inequality operator
bool operator !=(const ins_data& d1, const ins_data& d2)
{
 return ! (d1 == d2);
}


/// output stream operator for INS data
std::ostream&  operator<<(std::ostream& s, const ins_data& d)
{
  s << d.source_name << ", "
    << d.yaw << ", "
    << d.pitch << ", "
    << d.roll << ", "
    << d.lat << ", "
    << d.lon << ", "
    << d.alt << ", "
    << d.gps_sec << ", "
    << d.gps_week << ", "
    << d.n_vel << ", "
    << d.e_vel << ", "
    << d.up_vel << ", "
    << d.imu_status << ", "
    << d.local_adj << ", "
    << d.dst_flag << '\n';
  return s;
}

/// input stream operator for INS data
std::istream&  operator>>(std::istream& s, ins_data& d)
{
  std::string line;
  std::getline(s, line);

  std::stringstream ss(line);
  std::vector<std::string> tokens;
  std::string token;
  while(std::getline(ss, token, ','))
  {
    tokens.push_back(token);
  }

  // set the data to the defaults
  d = ins_data();

#define PARSE_FIELD(num, name) \
  if(tokens.size() > num) \
  { \
    ss.clear(); \
    ss.str(tokens[num]); \
    ss >> d.name; \
  }

  PARSE_FIELD(0, source_name);
  PARSE_FIELD(1, yaw);
  PARSE_FIELD(2, pitch);
  PARSE_FIELD(3, roll);
  PARSE_FIELD(4, lat);
  PARSE_FIELD(5, lon);
  PARSE_FIELD(6, alt);
  PARSE_FIELD(7, gps_sec);
  PARSE_FIELD(8, gps_week);
  PARSE_FIELD(9, n_vel);
  PARSE_FIELD(10, e_vel);
  PARSE_FIELD(11, up_vel);
  PARSE_FIELD(12, imu_status);
  PARSE_FIELD(13, local_adj);
  PARSE_FIELD(14, dst_flag);
#undef PARSE_FIELD

  return s;
}


} // end namespace maptk
