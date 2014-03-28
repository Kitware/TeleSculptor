/*ckwg +29
 * Copyright 2014 by Kitware, Inc.
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
 * \brief Header file for a map from frame IDs to cameras
 */

#ifndef MAPTK_CAMERA_MAP_H_
#define MAPTK_CAMERA_MAP_H_

#include "types.h"
#include "camera.h"
#include <map>
#include <boost/shared_ptr.hpp>

namespace maptk
{

/// An abstract mapping between frame IDs and cameras
class camera_map
{
public:
  /// typedef for std::map from integer IDs to cameras
  typedef std::map<frame_id_t, camera_sptr> map_camera_t;

  /// Destructor
  virtual ~camera_map() {}

  /// Return the number of cameras in the map
  virtual size_t size() const = 0;

  /// Return a map from integer IDs to camera shared pointers
  virtual map_camera_t cameras() const = 0;
};

/// typedef for a camera shared pointer
typedef boost::shared_ptr<camera_map> camera_map_sptr;


/// A concrete camera_map that simply wraps a std::map.
class simple_camera_map
: public camera_map
{
public:
  /// Default Constructor
  simple_camera_map() {}

  /// Constructor from a std::map of cameras
  explicit simple_camera_map(const map_camera_t& cameras)
  : data_(cameras) {}

  /// Return the number of cameras in the map
  virtual size_t size() const { return data_.size(); }

  /// Return a map from integer IDs to camera shared pointers
  virtual map_camera_t cameras() const { return data_; }

protected:

  /// The map from integer IDs to camera shared pointers
  map_camera_t data_;
};


} // end namespace maptk


#endif // MAPTK_CAMERA_MAP_H_
