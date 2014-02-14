/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include <test_common.h>
#include <test_random_point.h>

#include <maptk/vxl/register.h>
#include <maptk/vxl/estimate_homography.h>


maptk::matrix_3x3d sample_homography()
{
  maptk::matrix_3x3d H(0.0);
  H(0,0) = 2.0;
  H(1,1) = 3.0;
  H(2,2) = 1.0;
  H(0,2) = -1.5;
  H(1,2) = 5.0;
  return H;
}

#define TEST_ARGS ()

DECLARE_TEST_MAP();

int
main(int argc, char* argv[])
{
  CHECK_ARGS(1);

  maptk::vxl::register_algorithms();

  testname_t const testname = argv[1];

  RUN_TEST(testname);
}


IMPLEMENT_TEST(create)
{
  using namespace maptk;
  algo::estimate_homography_sptr est_H = algo::estimate_homography::create("vxl");
  if (!est_H)
  {
    TEST_ERROR("Unable to create vxl::estimate_homography by name");
  }
}


IMPLEMENT_TEST(not_enough_points)
{
  using namespace maptk;
  vxl::estimate_homography estimator;

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
  vxl::estimate_homography estimator;

  std::vector<vector_2d> pts1, pts2;
  std::vector<bool> inliers;

  pts1.push_back(vector_2d(0.0, 0.0));
  pts1.push_back(vector_2d(2.0, 0.0));
  pts1.push_back(vector_2d(0.0, 2.0));
  pts1.push_back(vector_2d(2.0, 2.0));

  matrix_3x3d true_H = sample_homography();

  // transform pts1 to pts2 using true_H
  for(unsigned i=0; i<pts1.size(); ++i)
  {
    vector_3d v = true_H * vector_3d(pts1[i].x(), pts1[i].y(), 1.0);
    pts2.push_back(vector_2d(v.x()/v.z(), v.y()/v.z()));
  }

  matrix_3x3d H = estimator.estimate(pts1, pts2, inliers);
  H /= H(2,2);

  double H_error = (true_H - H).frobenius_norm();
  std::cout << "Homography estimation error: "<< H_error << std::endl;

  TEST_NEAR("Frobenius norm between estimated and true homography",
            H_error, 0.0, 1e-14);
}


IMPLEMENT_TEST(ideal_points)
{
  using namespace maptk;
  vxl::estimate_homography estimator;

  matrix_3x3d true_H = sample_homography();

  // create random points that perfectly correspond via true_H
  std::vector<vector_2d> pts1, pts2;
  for(unsigned i=0; i<100; ++i)
  {
    vector_2d v2 = random_point2(1000.0) + 500.0;
    pts1.push_back(v2);
    vector_3d v3 = true_H * vector_3d(v2.x(), v2.y(), 1.0);
    pts2.push_back(vector_2d(v3.x()/v3.z(), v3.y()/v3.z()));
  }

  std::vector<bool> inliers;
  matrix_3x3d H = estimator.estimate(pts1, pts2, inliers);
  H /= H(2,2);

  double H_error = (true_H - H).frobenius_norm();
  std::cout << "Homography estimation error: "<< H_error << std::endl;
  TEST_NEAR("Frobenius norm between estimated and true homography",
            H_error, 0.0, 1e-8);

  unsigned num_inliers = static_cast<unsigned>(std::count(inliers.begin(),
                                                          inliers.end(), true));
  std::cout << "num inliers "<<num_inliers<<std::endl;
  TEST_EQUAL("All points are inliers", num_inliers, 100);
}


IMPLEMENT_TEST(noisy_points)
{
  using namespace maptk;
  vxl::estimate_homography estimator;

  matrix_3x3d true_H = sample_homography();

  // create random points + noise that approximately correspond via true_H
  std::vector<vector_2d> pts1, pts2;
  for(unsigned i=0; i<100; ++i)
  {
    vector_2d v2 = random_point2(1000.0) + 500.0;
    pts1.push_back(v2 + random_point2(0.1));
    vector_3d v3 = true_H * vector_3d(v2.x(), v2.y(), 1.0);
    pts2.push_back(vector_2d(v3.x()/v3.z(), v3.y()/v3.z())
                   + random_point2(0.1));
  }

  std::vector<bool> inliers;
  matrix_3x3d H = estimator.estimate(pts1, pts2, inliers);
  H /= H(2,2);

  double H_error = (true_H - H).frobenius_norm();
  std::cout << "Homography estimation error: "<< H_error << std::endl;
  TEST_NEAR("Frobenius norm between estimated and true homography",
            H_error, 0.0, 0.2);

  unsigned num_inliers = static_cast<unsigned>(std::count(inliers.begin(),
                                                          inliers.end(), true));
  std::cout << "num inliers "<<num_inliers<<std::endl;
  if (num_inliers < 90)
  {
    TEST_ERROR("Fewer than 90% of points detected as inliers");
  }
}


IMPLEMENT_TEST(outlier_points)
{
  using namespace maptk;
  vxl::estimate_homography estimator;

  matrix_3x3d true_H = sample_homography();

  // create random points + noise that approximately correspond via true_H
  std::vector<vector_2d> pts1, pts2;
  std::vector<bool> true_inliers;
  for(unsigned i=0; i<100; ++i)
  {
    vector_2d v2 = random_point2(1000.0) + 500.0;
    pts1.push_back(v2);
    vector_3d v3 = true_H * vector_3d(v2.x(), v2.y(), 1.0);
    pts2.push_back(vector_2d(v3.x()/v3.z(), v3.y()/v3.z()));
    true_inliers.push_back(true);
    if (i%3 == 0)
    {
      pts2.back() = random_point2(1000.0) + 500.0;
      true_inliers.back() = false;
    }
  }

  std::vector<bool> inliers;
  matrix_3x3d H = estimator.estimate(pts1, pts2, inliers);
  H /= H(2,2);

  double H_error = (true_H - H).frobenius_norm();
  std::cout << "Homography estimation error: "<< H_error << std::endl;
  TEST_NEAR("Frobenius norm between estimated and true homography",
            H_error, 0.0, 1e-8);

  unsigned num_inliers = static_cast<unsigned>(std::count(inliers.begin(),
                                                          inliers.end(), true));
  std::cout << "num inliers "<<num_inliers<<std::endl;

  unsigned correct_inliers = 0;
  for( unsigned i=0; i<inliers.size(); ++i )
  {
    if (true_inliers[i] == inliers[i])
    {
      ++correct_inliers;
    }
  }
  std::cout << "correct inliers "<<correct_inliers<<std::endl;

  if (correct_inliers < 90)
  {
    TEST_ERROR("Fewer than 90% of points have correct inlier status");
  }
}
