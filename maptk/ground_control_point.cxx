// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

/**
 * \file
 * \brief Implementation for
 * \link kwiver::vital::ground_control_point ground_control_point \endlink
 * objects
 */

#include "ground_control_point.h"
#include <vital/io/eigen_io.h>

namespace kwiver
{
namespace vital
{

//-----------------------------------------------------------------------------
ground_control_point::ground_control_point() = default;

//-----------------------------------------------------------------------------
ground_control_point::ground_control_point(vector_3d const& loc,
                                           std::string const& name)
  : loc_{loc}, name_{name}
{
}

//-----------------------------------------------------------------------------
std::ostream& operator<<(std::ostream& s, ground_control_point const& m)
{
  s << m.loc() << ' '
    << m.geo_loc() << ' ' << m.elevation() << ' '
    << m.name();
  return s;
}

//-----------------------------------------------------------------------------
// The following code does not work since geo_point doesn't have an input stream
// operator defined.
// std::istream& operator>>(std::istream& s, ground_control_point& pt)
// {
//   vector_3d loc;
//   geo_point geo_loc;
//   double elevation;
//   std::string name;

//   s >> loc >> geo_loc >> elevation >> name;
//   pt.set_loc(loc);
//   pt.set_geo_loc(geo_loc);
//   pt.set_elevation(elevation);
//   pt.set_name(name);
//   return s;
// }

} // end namespace vital
} // end namespace kwiver
