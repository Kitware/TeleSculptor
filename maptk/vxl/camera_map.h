/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief Header file for a map from frame IDs to vpgl cameras
 */

#ifndef MAPTK_VXL_CAMERA_MAP_H_
#define MAPTK_VXL_CAMERA_MAP_H_

#include <maptk/vxl/vxl_config.h>

#include <maptk/core/camera_map.h>
#include <map>
#include <vpgl/vpgl_perspective_camera.h>

namespace maptk
{

namespace vxl
{

/// A concrete camera_map that wraps a map of vpgl_perspective_camera
class MAPTK_VXL_EXPORT camera_map
: public maptk::camera_map
{
public:
  /// typedef for a map of frame numbers to vpgl_perspective_camera
  typedef std::map<unsigned int, vpgl_perspective_camera<double> > map_vcam_t;

  /// Default Constructor
  camera_map() {}

  /// Constructor from a std::map of vpgl_perspective_camera
  explicit camera_map(const map_vcam_t& cameras)
  : data_(cameras) {}

  /// Return the number of cameras in the map
  virtual size_t size() const { return data_.size(); }

  /// Return a map from integer IDs to camera shared pointers
  virtual map_camera_t cameras() const;

  /// Return underlying map from IDs to vpgl_perspective_camera
  virtual map_vcam_t vpgl_cameras() const { return data_; }

protected:

  /// The map from integer IDs to vpgl_perspective_camera
  map_vcam_t data_;
};


/// Convert any camera map to a vpgl camera map
MAPTK_VXL_EXPORT
camera_map::map_vcam_t
camera_map_to_vpgl(const maptk::camera_map& cam_map);


} // end namespace vxl

} // end namespace maptk


#endif // MAPTK_VXL_CAMERA_MAP_H_
