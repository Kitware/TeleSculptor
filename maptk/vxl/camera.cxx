/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */


/**
 * \file
 * \brief Implementation for conversions between maptk and vpgl cameras
 */

#include <maptk/vxl/camera.h>

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
  const vpgl_calibration_matrix<T>& vk = vcam.get_calibration();
  vgl_point_2d<T> vpp = vk.principal_point();
  mcam.set_intrinsics(camera_intrinsics_<T>(vk.focal_length() * vk.x_scale(),
                                            vector_2_<T>(vpp.x(), vpp.y()),
                                            vk.x_scale() / vk.y_scale(),
                                            vk.skew()));

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
  const camera_intrinsics_<T>& mk = mcam.get_intrinsics();
  const vector_2_<T>& mpp = mk.principal_point();
  vcam.set_calibration(vpgl_calibration_matrix<T>(mk.focal_length(),
                                                  vgl_point_2d<T>(mpp.x(),
                                                                  mpp.y()),
                                                  1, 1.0 / mk.aspect_ratio(),
                                                  mk.skew()));

  const vector_4_<T>& mr = mcam.get_rotation().quaternion();
  vcam.set_rotation(vgl_rotation_3d<T>(vnl_quaternion<T>(mr.x(), mr.y(),
                                                         mr.z(), mr.w())));

  const vector_3_<T>& mc = mcam.get_center();
  vcam.set_camera_center(vgl_point_3d<T>(mc.x(), mc.y(), mc.z()));
}


#define INSTANTIATE_CAMERA(T) \
template camera_sptr vpgl_camera_to_maptk(const vpgl_perspective_camera<T>& vcam); \
template void vpgl_camera_to_maptk(const vpgl_perspective_camera<T>&, camera_<T>&); \
template void maptk_to_vpgl_camera(const camera_<T>&, vpgl_perspective_camera<T>&)

INSTANTIATE_CAMERA(double);
INSTANTIATE_CAMERA(float);

#undef INSTANTIATE_CAMERA

} // end namespace vxl

} // end namespace maptk
