/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_CAMERA_MAP_H_
#define MAPTK_CAMERA_MAP_H_

/**
 * \file
 * \brief Header file for a map from frame IDs to cameras
 */

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
  typedef std::map<unsigned int, camera_sptr> map_camera_t;

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
