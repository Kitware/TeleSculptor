/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief core ins_data implementation
 */

#include "ins_data.h"

#include <vector>
#include <sstream>
#include <iomanip>

#include <maptk/core/exceptions/io.h>

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
std::ostream& operator<<(std::ostream& s, const ins_data& d)
{
  using std::setprecision;
  s << d.source_name << ", "
    << setprecision(12) << d.yaw << ", "
    << setprecision(12) << d.pitch << ", "
    << setprecision(12) << d.roll << ", "
    << setprecision(12) << d.lat << ", "
    << setprecision(12) << d.lon << ", "
    << setprecision(12) << d.alt << ", "
    << setprecision(12) << d.gps_sec << ", "
    << d.gps_week << ", "
    << setprecision(12) << d.n_vel << ", "
    << setprecision(12) << d.e_vel << ", "
    << setprecision(12) << d.up_vel << ", "
    << d.imu_status << ", "
    << d.local_adj << ", "
    << d.dst_flag << '\n';
  return s;
}

/// input stream operator for INS data
std::istream& operator>>(std::istream& s, ins_data& d)
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

  /// \cond DoxygenSuppress
#define PARSE_FIELD(num, name) \
  if(tokens.size() > num) \
  { \
    ss.clear(); \
    ss.str(tokens[num]); \
    ss >> d.name; \
  }
  /// \endcond

  unsigned int base=0;
  // some POS files do not have the source name
  if( tokens.size() < 14 || tokens.size() > 15)
  {
    std::ostringstream ss;
    ss << "Too few fields found in the given data stream "
       << "(discovered " << tokens.size() << " field(s), expected "
       << "14 or 15).";
    throw invalid_data(ss.str());
  }
  else if( tokens.size() == 15 )
  {
    PARSE_FIELD(0, source_name);
    base = 1;
  }

  PARSE_FIELD(base+0, yaw);
  PARSE_FIELD(base+1, pitch);
  PARSE_FIELD(base+2, roll);
  PARSE_FIELD(base+3, lat);
  PARSE_FIELD(base+4, lon);
  PARSE_FIELD(base+5, alt);
  PARSE_FIELD(base+6, gps_sec);
  PARSE_FIELD(base+7, gps_week);
  PARSE_FIELD(base+8, n_vel);
  PARSE_FIELD(base+9, e_vel);
  PARSE_FIELD(base+10, up_vel);
  PARSE_FIELD(base+11, imu_status);
  PARSE_FIELD(base+12, local_adj);
  PARSE_FIELD(base+13, dst_flag);
#undef PARSE_FIELD

  return s;
}


} // end namespace maptk
