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
 * \brief Header file for conversions between maptk and vpgl cameras
 */

#ifndef MAPTK_VXL_CAMERA_H_
#define MAPTK_VXL_CAMERA_H_

#include <maptk/camera.h>
#include <maptk/plugins/vxl/vxl_config.h>

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


/// Convert a vpgl_calibration_matrix to a maptk::camera_intrinsics_
template <typename T>
MAPTK_VXL_EXPORT
void vpgl_calibration_to_maptk(const vpgl_calibration_matrix<T>& vcal,
                               camera_intrinsics_<T>& mcal);

/// Convert a maptk::camera_intrinsics_ to a vpgl_calibration_matrix
template <typename T>
MAPTK_VXL_EXPORT
void maptk_to_vpgl_calibration(const camera_intrinsics_<T>& mcal,
                               vpgl_calibration_matrix<T>& vcal);



} // end namespace vxl

} // end namespace maptk


#endif // MAPTK_VXL_CAMERA_H_
