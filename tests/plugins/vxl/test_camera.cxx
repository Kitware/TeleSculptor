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
 * \brief test VXL camera class functionality
 */

#include <test_common.h>

#include <cstdio>

#include <maptk/plugins/vxl/camera.h>

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
  vpgl_calibration_matrix<T> vk(T(4000), vgl_point_2d<T>(300,400),
                                T(1.0), T(0.75), T(0.0001));
  vgl_rotation_3d<T> vr(T(0.7), T(0.1), T(1.3));
  vgl_point_3d<T> vc(T(100.0), T(0.0), T(20.0));
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
  test_convert_camera<float>(1e-7f);
}
