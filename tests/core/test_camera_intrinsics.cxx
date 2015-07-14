/*ckwg +29
 * Copyright 2014-2015 by Kitware, Inc.
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
 * \brief core camera_intrinsics tests
 */

#include <test_common.h>

#include <maptk/camera_intrinsics.h>

#define TEST_ARGS ()

DECLARE_TEST_MAP();

int
main(int argc, char* argv[])
{
  CHECK_ARGS(1);

  testname_t const testname = argv[1];

  RUN_TEST(testname);
}


IMPLEMENT_TEST(map)
{
  using namespace maptk;
  vector_2d pp(300,400);
  double f = 1000.0;
  double a = 0.75;
  double s = 0.1;
  camera_intrinsics_d K(f, pp, a, s);

  vector_2d origin = K.map(vector_2d(0,0));

  TEST_NEAR("(0,0) maps to principal point",
            (origin-pp).norm(), 0.0, 1e-12);

  TEST_NEAR("principal point unmaps to (0,0)",
            K.unmap(pp).norm(), 0.0, 1e-12);

  vector_2d test_pt(1,2);
  vector_2d mapped_test = K.map(test_pt);
  vector_2d mapped_test_gt(test_pt.x() * f + test_pt.y() * s + pp.x(),
                           test_pt.y() * f / a + pp.y());
  TEST_NEAR("mapped test point at GT location",
            (mapped_test - mapped_test_gt).norm(), 0.0, 1e-12);

  vector_2d unmapped = K.unmap(mapped_test);
  TEST_NEAR("unmap is the inverse of map",
            (unmapped-test_pt).norm(), 0.0, 1e-12);

  vector_3d homg_pt(2.5 * vector_3d(test_pt.x(), test_pt.y(), 1));
  vector_2d mapped_from_3d = K.map(homg_pt);
  TEST_NEAR("mapped 3D test point at GT location",
            (mapped_from_3d - mapped_test_gt).norm(), 0.0, 1e-12);

}


IMPLEMENT_TEST(distort)
{
  using namespace maptk;
  Eigen::VectorXd d(2);
  d << 0.01 , 0.002;
  camera_intrinsics_d K;
  K.set_dist_coeffs(d);

  vector_2d origin(0,0);

  TEST_NEAR("(0,0) is not distorted",
            K.distort(origin).norm(), 0.0, 1e-12);

  TEST_NEAR("undistort origin",
            K.undistort(origin).norm(), 0.0, 1e-12);

  vector_2d test_pt(1,2);
  vector_2d distorted_test = K.distort(test_pt);
  std::cout << "test point: "<< test_pt.transpose() << "\n"
            << "distorted: " << distorted_test.transpose() << std::endl;
  double r2 = test_pt.squaredNorm();
  vector_2d distorted_test_gt = test_pt * (1.0 + r2*d[0] + r2*r2*d[1]);
  TEST_NEAR("distorted test point at GT location",
            (distorted_test - distorted_test_gt).norm(), 0.0, 1e-12);

  vector_2d undistorted = K.unmap(distorted_test);
  std::cout << "undistorted: "<< undistorted.transpose() << std::endl;
  TEST_NEAR("undistort is the inverse of distort",
            (undistorted-test_pt).norm(), 0.0, 1e-12);
}
