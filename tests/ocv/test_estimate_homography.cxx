/*ckwg +5
 * Copyright 2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include <test_common.h>

#include <maptk/ocv/register.h>
#include <maptk/ocv/estimate_homography.h>



#define TEST_ARGS ()

DECLARE_TEST_MAP();

int
main(int argc, char* argv[])
{
  CHECK_ARGS(1);

  maptk::ocv::register_algorithms();

  testname_t const testname = argv[1];

  RUN_TEST(testname);
}


IMPLEMENT_TEST(create)
{
  using namespace maptk;
  algo::estimate_homography_sptr est_H = algo::estimate_homography::create("ocv");
  if (!est_H)
  {
    TEST_ERROR("Unable to create ocv::estimate_homography by name");
  }
}


IMPLEMENT_TEST(not_enough_points)
{
  using namespace maptk;
  ocv::estimate_homography estimator;

  std::vector<vector_2d> pts1, pts2;
  std::vector<bool> inliers;

  matrix_3x3d H = estimator.estimate(pts1, pts2, inliers);
  if (H != matrix_3x3d(0.0))
  {
    TEST_ERROR("Estimation with no points should produce a zero matrix");
  }

  pts1.push_back(vector_2d(0.0, 0.0));
  pts1.push_back(vector_2d(2.0, 0.0));
  pts1.push_back(vector_2d(0.0, 2.0));

  pts2.push_back(vector_2d(1.0, 1.0));
  pts2.push_back(vector_2d(4.0, 1.0));
  pts2.push_back(vector_2d(1.0, 4.0));

  H = estimator.estimate(pts1, pts2, inliers);
  if (H != matrix_3x3d(0.0))
  {
    TEST_ERROR("Estimation with < 4 points should produce a zero matrix");
  }
}


IMPLEMENT_TEST(four_points)
{
  using namespace maptk;
  ocv::estimate_homography estimator;

  std::vector<vector_2d> pts1, pts2;
  std::vector<bool> inliers;

  pts1.push_back(vector_2d(0.0, 0.0));
  pts1.push_back(vector_2d(2.0, 0.0));
  pts1.push_back(vector_2d(0.0, 2.0));
  pts1.push_back(vector_2d(2.0, 2.0));

  matrix_3x3d true_H(0.0);
  true_H(0,0) = 2.0;
  true_H(1,1) = 3,0;
  true_H(2,2) = 1.0;
  true_H(0,2) = -1.5;
  true_H(1,2) = 5.0;

  // transform pts1 to pts2 using true_H
  for(unsigned i=0; i<pts1.size(); ++i)
  {
    vector_3d v = true_H * vector_3d(pts1[i].x(), pts1[i].y(), 1.0);
    pts2.push_back(vector_2d(v.x()/v.z(), v.y()/v.z()));
  }

  matrix_3x3d H = estimator.estimate(pts1, pts2, inliers);

  std::cout << "H = "<<true_H <<std::endl;
  TEST_NEAR("Frobenius norm between estimated and true homography",
            (true_H - H).frobenius_norm(), 0.0, 1e-14);
}


IMPLEMENT_TEST(apply_to_points)
{
  using namespace maptk;
  ocv::estimate_homography estimator;

  std::vector<vector_2d> pts1, pts2;
  std::vector<bool> inliers;

  matrix_3x3d H = estimator.estimate(pts1, pts2, inliers);
}
