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
#include <test_random_point.h>

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


// helper function to distort a point
maptk::vector_2d
distort_point(const maptk::vector_2d& pt,
           const Eigen::VectorXd& d)
{
  using namespace maptk;
  const double x2 = pt.x() * pt.x();
  const double y2 = pt.y() * pt.y();
  const double r2 = x2 + y2;
  const double r4 = r2*r2;
  const double r6 = r2*r4;
  const double two_xy = 2 * pt.x() * pt.y();

  double scale = 1.0;
  vector_2d dist_pt = pt;
  if(d.rows() > 0)
  {
    scale += r2*d[0];
    if(d.rows() > 1)
    {
      scale += r4*d[1];
      if(d.rows() > 4)
      {
        scale += r6*d[4];
        if(d.rows() > 7)
        {
          scale /= 1.0 + r2*d[5] + r4*d[6] + r6*d[7];
        }
      }
    }
  }
  dist_pt *= scale;
  if(d.rows() > 3)
  {
    dist_pt += vector_2d(d[2]*two_xy + d[3]*(r2 + 2*x2),
                         d[3]*two_xy + d[2]*(r2 + 2*y2));
  }
  return dist_pt;
}



void test_distortion(const Eigen::VectorXd& d)
{
  using namespace maptk;
  camera_intrinsics_d K;
  K.set_dist_coeffs(d);

  vector_2d origin(0,0);

  TEST_NEAR("(0,0) is not distorted",
            K.distort(origin).norm(), 0.0, 1e-12);

  TEST_NEAR("undistort origin",
            K.undistort(origin).norm(), 0.0, 1e-12);

  for(unsigned int i = 1; i<100; ++i)
  {
    vector_2d test_pt = testing::random_point2d(0.5);
    // the distortion model is only valid within about a unit distance
    // from the origin in normalized coordinates
    // project distant points back onto the unit circle
    if (test_pt.norm() > 1.0)
    {
      test_pt.normalize();
    }
    vector_2d distorted_test = K.distort(test_pt);
    std::cout << "test point: "<< test_pt.transpose() << "\n"
              << "distorted: " << distorted_test.transpose() << std::endl;

    vector_2d distorted_test_gt = distort_point(test_pt, d);
    TEST_NEAR("distorted test point at GT location",
              (distorted_test - distorted_test_gt).norm(), 0.0, 1e-12);

    vector_2d undistorted = K.undistort(distorted_test);
    std::cout << "undistorted: "<< undistorted.transpose() << std::endl;
    TEST_NEAR("undistort is the inverse of distort",
              (undistorted-test_pt).norm(), 0.0, 1e-10);
  }
}


IMPLEMENT_TEST(distort_r1)
{
  Eigen::VectorXd d(1);
  d << -0.01;
  test_distortion(d);
}


IMPLEMENT_TEST(distort_r2)
{
  Eigen::VectorXd d(2);
  d << -0.03, 0.007;
  test_distortion(d);
}


IMPLEMENT_TEST(distort_r3)
{
  Eigen::VectorXd d(5);
  d << -0.03, 0.01, 0, 0, -0.02;
  test_distortion(d);
}


IMPLEMENT_TEST(distort_tang_r3)
{
  Eigen::VectorXd d(5);
  d << -0.03, 0.01, -1e-3, 5e-4, -0.02;
  test_distortion(d);
}


IMPLEMENT_TEST(distort_tang_r6)
{
  Eigen::VectorXd d(8);
  d << -0.03, 0.01, -1e-3, 5e-4, -0.02, 1e-4, -2e-3, 3e-4;
  test_distortion(d);
}
