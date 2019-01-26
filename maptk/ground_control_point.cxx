/*ckwg +29
 * Copyright 2019 by Kitware, Inc.
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
 *  * Neither name of Kitware, Inc. nor the names of any contributors may be
 * used to endorse or promote products derived from this software without
 * specific prior written permission.
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

/// Default Constructor
ground_control_point::ground_control_point()
  : loc_(0, 0, 0)
  , geo_loc_()
  , elevation_(0)
  , name_()
{
}

/// Constructor for a ground_control_point
ground_control_point::ground_control_point(vector_3d const& loc,
                                           std::string const& name)
  : loc_(loc)
  , geo_loc_()
  , elevation_(0)
  , name_(name)
{
}

/// Constructor for a ground_control_point from a ground_control_point
ground_control_point::ground_control_point(ground_control_point const& other)
  : loc_(other.loc())
  , geo_loc_(other.geo_loc())
  , elevation_(other.elevation())
  , name_(other.name())
{
}

/// output stream operator for a ground_control_point
std::ostream& operator<<(std::ostream& s, ground_control_point const& m)
{
  s << m.loc() << " " << m.geo_loc() << " " << m.elevation() << " " << m.name();
  return s;
}

/// input stream operator for a ground_control_point
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
