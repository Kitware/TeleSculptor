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
 * \brief Header for
 * \link kwiver::vital::ground_control_point ground_control_point \endlink
 * objects
 */

#ifndef MAPTK_GROUND_CONTROL_POINT_H_
#define MAPTK_GROUND_CONTROL_POINT_H_

#include <maptk/maptk_export.h>

#include <vital/types/geo_point.h>
#include <vital/vital_export.h>

#include <iostream>
#include <map>
#include <memory>
#include <string>

namespace kwiver
{
namespace vital
{

/// alias for ground control point id
using ground_control_point_id_t = uint32_t;

/// A representation of a 3D ground control point.
class MAPTK_EXPORT ground_control_point
{
public:
  /// Constructor
  ground_control_point();
  ground_control_point(vector_3d const& loc,
                       std::string const& name = std::string());
  ground_control_point(ground_control_point const& other);
  /// Destructor
  ~ground_control_point() = default;

  /// Accessor for the world coordinates
  vector_3d loc() const
  {
    return loc_;
  }
  /// Set the world location
  void set_loc(vector_3d const& loc)
  {
    loc_ = loc;
  }
  /// Accessor for the geodetic location
  geo_point geo_loc() const
  {
    return geo_loc_;
  }
  /// Set the geodetic location
  void set_geo_loc(geo_point const& geo_loc)
  {
    geo_loc_ = geo_loc;
  }
  /// Accessor for the elevation of the ground control point
  double elevation() const
  {
    return elevation_;
  }
  /// Set the elevation for the ground control point
  void set_elevation(double elev)
  {
    elevation_ = elev;
  }
  /// Overload to set geo_loc and elevation at the same time
  void set_geo_loc(geo_point const& geo_loc, double elev)
  {
    geo_loc_ = geo_loc;
    elevation_ = elev;
  }
  /// Accessor for the name of the ground control point
  std::string name() const
  {
    return name_;
  }
  /// Set the name of the ground control point
  void set_name(std::string const& name)
  {
    name_ = name;
  }

protected:
  vector_3d loc_;
  geo_point geo_loc_;
  double elevation_;
  std::string name_;
};

/// output stream operator for a ground control point
/**
 * \param s output stream
 * \param m ground_control_point to stream
 */

/// output stream operator for a ground_control_point
VITAL_EXPORT std::ostream& operator<<(std::ostream& s,
                                      ground_control_point const& m);

/// input stream operator for a ground_control_point
// The input stream operator is commented since geo_point doesn't have an
// input stream operator defined blocking this class' operator
// VITAL_EXPORT std::istream& operator>>(std::istream& s, ground_control_point&
// m);

/// alias for a ground_control_point shared pointer
using ground_control_point_sptr = std::shared_ptr<ground_control_point>;

// ----------------------------------------------------------------------------
/// A mapping between IDs and ground control points
class MAPTK_EXPORT ground_control_point_map
{
public:
  /// alias for std::map from integer IDs to ground control points
  using map_ground_control_point_t =
    std::map<ground_control_point_id_t, ground_control_point_sptr>;

  /// Constructor
  explicit ground_control_point_map(const map_ground_control_point_t& m)
    : data_(m)
  {
  }

  /// Destructor
  ~ground_control_point_map() = default;

  /// Return the number of ground control points in the map
  size_t size() const
  {
    return data_.size();
  }

  /// Return a map from integer IDs to ground control point shared pointers
  map_ground_control_point_t ground_control_points() const
  {
    return data_;
  }

protected:
  /// The map from integer IDs to ground control point shared pointers
  map_ground_control_point_t data_;
};

/// alias for a ground control point shared pointer
using ground_control_point_map_sptr = std::shared_ptr<ground_control_point_map>;

} // end namespace vital
} // end namespace kwiver

#endif
