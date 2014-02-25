/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief Header file for conversions between maptk and vpgl cameras
 */

#ifndef MAPTK_VXL_CAMERA_H_
#define MAPTK_VXL_CAMERA_H_

#include <maptk/vxl/vxl_config.h>

#include <maptk/core/camera.h>
#include <vpgl/vpgl_perspective_camera.h>

namespace maptk
{

namespace vxl
{

/// Construct a camera_sptr from a vpgl_perspective_camera
template <typename T>
MAPTK_VXL_EXPORT
camera_sptr vpgl_camera_to_maptk(const vpgl_perspective_camera<T>& vcam);


/// Convert a vpgl_perspective_camera to a maptk::camera_
template <typename T>
MAPTK_VXL_EXPORT
void vpgl_camera_to_maptk(const vpgl_perspective_camera<T>& vcam,
                          camera_<T>& mcam);

/// Convert a maptk::camera_ to a vpgl_perspective_camera
template <typename T>
MAPTK_VXL_EXPORT
void maptk_to_vpgl_camera(const camera_<T>& mcam,
                          vpgl_perspective_camera<T>& vcam);


} // end namespace vxl

} // end namespace maptk


#endif // MAPTK_VXL_CAMERA_H_
