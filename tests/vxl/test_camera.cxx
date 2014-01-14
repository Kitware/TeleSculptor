/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include <test_common.h>

#include <cstdio>
#include <maptk/vxl/camera.h>

#define TEST_ARGS ()

DECLARE_TEST_MAP();

int
main(int argc, char* argv[])
{
  CHECK_ARGS(1);

  testname_t const testname = argv[1];

  RUN_TEST(testname);
}


template <typename T>
vpgl_perspective_camera<T> sample_vpgl_camera()
{
  using namespace maptk;
  vpgl_calibration_matrix<T> vk(4000.0, vgl_point_2d<T>(300,400),
                                1.0, 0.75, 0.0001);
  vgl_rotation_3d<T> vr(0.7, 0.1, 1.3);
  vgl_point_3d<T> vc(100.0, 0.0, 20.0);
  return vpgl_perspective_camera<T>(vk, vc, vr);
}


IMPLEMENT_TEST(convert_camera_sptr)
{
  using namespace maptk;
  vpgl_perspective_camera<double> vcd = sample_vpgl_camera<double>();
  vpgl_perspective_camera<float> vcf = sample_vpgl_camera<float>();

  camera_sptr cam = vxl::vpgl_camera_to_maptk(vcd);
  if( !cam || cam->data_type() != typeid(double) )
  {
    TEST_ERROR("Type of converted camera does not match \"double\"");
  }

  cam = vxl::vpgl_camera_to_maptk(vcf);
  if( !cam || cam->data_type() != typeid(float) )
  {
    TEST_ERROR("Type of converted camera does not match \"float\"");
  }
}


template <typename T>
void test_convert_camera(T eps)
{
  using namespace maptk;
  vpgl_perspective_camera<T> vcam = sample_vpgl_camera<T>();

  camera_<T> mcam;
  vxl::vpgl_camera_to_maptk(vcam, mcam);

  vpgl_perspective_camera<T> vcam2;
  vxl::maptk_to_vpgl_camera(mcam, vcam2);

  double err = (vcam.get_matrix() - vcam2.get_matrix()).frobenius_norm();
  std::cout << "Camera matrix difference: "<<err<<std::endl;
  TEST_NEAR("Camera converted vpgl-->maptk-->vpgl", err, 0.0, eps);
}


IMPLEMENT_TEST(convert_camera_double)
{
  test_convert_camera<double>(1e-15);
}


IMPLEMENT_TEST(convert_camera_float)
{
  test_convert_camera<float>(1e-7);
}
