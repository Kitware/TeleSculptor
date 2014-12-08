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
 * \brief Implementation for conversions between maptk and vpgl cameras
 */

#include "camera.h"


namespace maptk
{

namespace vxl
{


/// Construct a camera_sptr from a vpgl_perspective_camera
template <typename T>
camera_sptr
vpgl_camera_to_maptk(const vpgl_perspective_camera<T>& vcam)
{
  camera_<T>* mcam = new camera_<T>;
  vpgl_camera_to_maptk(vcam, *mcam);
  return camera_sptr(mcam);
}


/// Convert a vpgl_perspective_camera to a maptk::camera_
template <typename T>
void
vpgl_camera_to_maptk(const vpgl_perspective_camera<T>& vcam,
                     camera_<T>& mcam)
{
  camera_intrinsics_<T> mk;
  vpgl_calibration_to_maptk(vcam.get_calibration(), mk);
  mcam.set_intrinsics(mk);

  const vnl_quaternion<T>& vr = vcam.get_rotation().as_quaternion();
  mcam.set_rotation(rotation_<T>(vector_4_<T>(vr.x(), vr.y(), vr.z(), vr.r())));

  const vgl_point_3d<T>& vc = vcam.get_camera_center();
  mcam.set_center(vector_3_<T>(vc.x(), vc.y(), vc.z()));
}


/// Convert a maptk::camera_ to a vpgl_perspective_camera
template <typename T>
void
maptk_to_vpgl_camera(const camera_<T>& mcam,
                     vpgl_perspective_camera<T>& vcam)
{
  vpgl_calibration_matrix<T> vk;
  maptk_to_vpgl_calibration(mcam.get_intrinsics(), vk);
  vcam.set_calibration(vk);

  const vector_4_<T>& mr = mcam.get_rotation().quaternion();
  vcam.set_rotation(vgl_rotation_3d<T>(vnl_quaternion<T>(mr.x(), mr.y(),
                                                         mr.z(), mr.w())));

  const vector_3_<T>& mc = mcam.get_center();
  vcam.set_camera_center(vgl_point_3d<T>(mc.x(), mc.y(), mc.z()));
}


/// Convert a vpgl_calibration_matrix to a maptk::camera_intrinsics_
template <typename T>
void
vpgl_calibration_to_maptk(const vpgl_calibration_matrix<T>& vcal,
                          camera_intrinsics_<T>& mcal)
{
  vgl_point_2d<T> vpp = vcal.principal_point();
  mcal = camera_intrinsics_<T>(vcal.focal_length() * vcal.x_scale(),
                               vector_2_<T>(vpp.x(), vpp.y()),
                               vcal.x_scale() / vcal.y_scale(),
                               vcal.skew());
}


/// Convert a maptk::camera_intrinsics_ to a vpgl_calibration_matrix
template <typename T>
void
maptk_to_vpgl_calibration(const camera_intrinsics_<T>& mcal,
                          vpgl_calibration_matrix<T>& vcal)
{
  const vector_2_<T>& mpp = mcal.principal_point();
  vcal = vpgl_calibration_matrix<T>(mcal.focal_length(),
                                    vgl_point_2d<T>(mpp.x(), mpp.y()),
                                    1, T(1) / mcal.aspect_ratio(),
                                    mcal.skew());
}



/// \cond DoxygenSuppress
#define INSTANTIATE_CAMERA(T) \
template MAPTK_VXL_EXPORT camera_sptr vpgl_camera_to_maptk(const vpgl_perspective_camera<T>& vcam); \
template MAPTK_VXL_EXPORT void vpgl_camera_to_maptk(const vpgl_perspective_camera<T>&, camera_<T>&); \
template MAPTK_VXL_EXPORT void maptk_to_vpgl_camera(const camera_<T>&, vpgl_perspective_camera<T>&); \
template MAPTK_VXL_EXPORT void vpgl_calibration_to_maptk(const vpgl_calibration_matrix<T>& vcal, \
                                                         camera_intrinsics_<T>& mcal); \
template MAPTK_VXL_EXPORT void maptk_to_vpgl_calibration(const camera_intrinsics_<T>& mcal, \
                                                         vpgl_calibration_matrix<T>& vcal)

INSTANTIATE_CAMERA(double);
INSTANTIATE_CAMERA(float);

#undef INSTANTIATE_CAMERA
/// \endcond

} // end namespace vxl

} // end namespace maptk
