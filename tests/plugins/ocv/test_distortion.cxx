/*ckwg +29
 * Copyright 2015 by Kitware, Inc.
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
 * \brief tests comparing MAP-Tk lens distortion to OpenCV
 */

#include <test_common.h>
#include <test_random_point.h>

#include <vital/types/camera_intrinsics.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#define TEST_ARGS ()

DECLARE_TEST_MAP();

int
main(int argc, char* argv[])
{
  CHECK_ARGS(1);

  testname_t const testname = argv[1];

  RUN_TEST(testname);
}

using namespace kwiver::vital;

void test_distortion(const Eigen::VectorXd& d)
{
  using namespace kwiver::maptk;
  simple_camera_intrinsics K;
  K.set_dist_coeffs(d.transpose());

  cv::Mat cam_mat;
  cv::eigen2cv(K.as_matrix(), cam_mat);
  int rows = d.rows() < 4 ? 4 : d.rows();
  cv::Mat dist = cv::Mat::zeros(1, rows, CV_64F);
  for(unsigned int i=0; i<d.rows(); ++i)
  {
    dist.at<double>(i) = d[i];
  }
  cv::Mat tvec = cv::Mat::zeros(1,3,CV_64F);
  cv::Mat rvec = tvec;

  std::cout << "K = " << cam_mat << "\nd = " << dist <<std::endl;

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
    std::vector<cv::Point3d> in_pts;
    std::vector<cv::Point2d> out_pts;
    in_pts.push_back(cv::Point3d(test_pt.x(), test_pt.y(), 1.0));
    cv::projectPoints(in_pts, rvec, tvec, cam_mat, dist, out_pts);
    vector_2d cv_distorted_test(out_pts[0].x, out_pts[0].y);

    std::cout << "OpenCV distorted: " << cv_distorted_test.transpose()
              << std::endl;

    TEST_NEAR("OpenCV and MAP-TK distortion match",
              (distorted_test-cv_distorted_test).norm(), 0.0, 1e-10);
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
