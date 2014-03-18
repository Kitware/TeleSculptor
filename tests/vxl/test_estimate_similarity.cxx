/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include <iostream>
#include <vector>

#include <test_common.h>
#include <test_random_point.h>

#include <maptk/core/exceptions.h>
#include <maptk/core/rotation.h>
#include <maptk/core/similarity.h>
#include <maptk/core/vector.h>
#include <maptk/vxl/register.h>
#include <maptk/vxl/estimate_similarity_transform.h>

#include <boost/foreach.hpp>


#define TEST_ARGS ()
DECLARE_TEST_MAP();

int main(int argc, char* argv[])
{
  CHECK_ARGS(1);
  maptk::vxl::register_algorithms();
  testname_t const testname = argv[1];
  RUN_TEST(testname);
}


IMPLEMENT_TEST(creation)
{
  using namespace maptk::algo;
  estimate_similarity_transform_sptr est_ST = estimate_similarity_transform::create("vxl");
  if (!est_ST)
  {
    TEST_ERROR("Unable to create vxl::estimate_similarity_transform by name");
  }
}


IMPLEMENT_TEST(not_enough_points)
{
  using namespace maptk;
  vxl::estimate_similarity_transform est_ST;
  std::vector<vector_3d> from, to;
  EXPECT_EXCEPTION(
    algorithm_exception,
    est_ST.estimate_transform(from, to),
    "estimating with zero points");
}


IMPLEMENT_TEST(uneven_sets)
{
  using namespace maptk;
  vxl::estimate_similarity_transform est_ST;
  std::vector<vector_3d> from, to;
  vector_3d dummy_vec(0,0,0);

  from.push_back(dummy_vec);
  from.push_back(dummy_vec);
  to.push_back(dummy_vec);

  EXPECT_EXCEPTION(
    algorithm_exception,
    est_ST.estimate_transform(from, to),
    "estimating with uneven sets");
}


IMPLEMENT_TEST(reprojection_100pts)
{
  // Process:
  //  1) generated points
  //  2) make up similarity transform and transform points
  //  3) generate similarity transform via estimation between two points sets
  //  4) check difference between crafted and estimated similarity transforms

  using namespace std;
  using namespace maptk;

  cerr << "Constructing 100 original, random points (std dev: 1.0)" << endl;
  vector<vector_3d> original_points;
  for (int i=0; i < 100; ++i)
  {
    original_points.push_back(testing::random_point3d(1.0));
  }

  cerr << "Constructing crafted similarity transformation" << endl;
  similarity_d m_sim(5.623,
                     rotation_d(vector_3d(-1.4, 0.23, 1.7)),
                     vector_3d(2.24, 1.51, 4.23));

  cerr << "Transforming original points by crafted transformation" << endl;
  vector<vector_3d> transformed_points;
  BOOST_FOREACH(vector_3d o_vec, original_points)
  {
    transformed_points.push_back(m_sim * o_vec);
  }

  cerr << "Estimating similarity transformation between point sets" << endl;
  vxl::estimate_similarity_transform est_ST;
  similarity_d e_sim = est_ST.estimate_transform(original_points,
                                                 transformed_points);

  cerr << "Original Transform : " << m_sim << endl
       << "Estimated Transform: " << e_sim << endl
       << "Euclidian norm     : " << (matrix_4x4d(m_sim) - matrix_4x4d(e_sim)).frobenius_norm() << endl;

  TEST_NEAR("Crafted and estimated similarity transforms match",
            (matrix_4x4d(m_sim) - matrix_4x4d(e_sim)).frobenius_norm(),
            0.0, 1e-14);

  cerr << "Constructing crafted similarity transformation WITH ZERO TRANSLATION" << endl;
  m_sim = similarity_d(5.623,
                       rotation_d(vector_3d(-1.4, 0.23, 1.7)),
                       vector_3d(0, 0, 0));

  cerr << "Transforming original points by crafted transformation" << endl;
  transformed_points.clear();
  BOOST_FOREACH(vector_3d o_vec, original_points)
  {
    transformed_points.push_back(m_sim * o_vec);
  }

  cerr << "Estimating similarity transformation between point sets" << endl;
  e_sim = est_ST.estimate_transform(original_points, transformed_points);

  cerr << "Original Transform : " << m_sim << endl
       << "Estimated Transform: " << e_sim << endl
       << "Euclidian norm     : " << (matrix_4x4d(m_sim) - matrix_4x4d(e_sim)).frobenius_norm() << endl;

  TEST_NEAR("Crafted and estimated similarity transforms match",
            (matrix_4x4d(m_sim) - matrix_4x4d(e_sim)).frobenius_norm(),
            0.0, 1e-14);
}


IMPLEMENT_TEST(reprojection_4pts)
{
  // Process:
  //  1) generated points
  //  2) make up similarity transform and transform points
  //  3) generate similarity transform via estimation between two points sets
  //  4) check difference between crafted and estimated similarity transforms

  using namespace std;
  using namespace maptk;

  cerr << "Constructing 4 original, random points (std dev: 1.0)" << endl;
  vector<vector_3d> original_points;
  cerr << "Random points:" << endl;
  for (int i=0; i < 4; ++i)
  {
    original_points.push_back(testing::random_point3d(1.0));
    cerr << "\t" << original_points.back() << endl;
  }

  cerr << "Constructing crafted similarity transformation" << endl;
  similarity_d m_sim(5.623,
                     rotation_d(vector_3d(-1.4, 0.23, 1.7)),
                     vector_3d(2.24, 1.51, 4.23));

  cerr << "Transforming original points by crafted transformation" << endl;
  vector<vector_3d> transformed_points;
  BOOST_FOREACH(vector_3d o_vec, original_points)
  {
    transformed_points.push_back(m_sim * o_vec);
  }

  cerr << "Estimating similarity transformation between point sets" << endl;
  vxl::estimate_similarity_transform est_ST;
  similarity_d e_sim = est_ST.estimate_transform(original_points,
                                                 transformed_points);

  cerr << "Original Transform : " << m_sim << endl
       << "Estimated Transform: " << e_sim << endl
       << "Euclidian norm     : " << (matrix_4x4d(m_sim) - matrix_4x4d(e_sim)).frobenius_norm() << endl;

  TEST_NEAR("Crafted and estimated similarity transforms match",
            (matrix_4x4d(m_sim) - matrix_4x4d(e_sim)).frobenius_norm(),
            0.0, 1e-12);
}


IMPLEMENT_TEST(reprojection_3pts)
{
  // Process:
  //  1) generated points
  //  2) make up similarity transform and transform points
  //  3) generate similarity transform via estimation between two points sets
  //  4) check difference between crafted and estimated similarity transforms

  using namespace std;
  using namespace maptk;

  cerr << "Constructing 3 original, random points (std dev: 1.0)" << endl;
  vector<vector_3d> original_points;
  cerr << "Random points:" << endl;
  for (int i=0; i < 3; ++i)
  {
    original_points.push_back(testing::random_point3d(1.0));
    cerr << "\t" << original_points.back() << endl;
  }

  cerr << "Constructing crafted similarity transformation" << endl;
  similarity_d m_sim(5.623,
                     rotation_d(vector_3d(-1.4, 0.23, 1.7)),
                     vector_3d(2.24, 1.51, 4.23));

  cerr << "Transforming original points by crafted transformation" << endl;
  vector<vector_3d> transformed_points;
  BOOST_FOREACH(vector_3d o_vec, original_points)
  {
    transformed_points.push_back(m_sim * o_vec);
  }

  cerr << "Estimating similarity transformation between point sets" << endl;
  vxl::estimate_similarity_transform est_ST;
  similarity_d e_sim = est_ST.estimate_transform(original_points,
                                                 transformed_points);

  cerr << "Original Transform : " << m_sim << endl
       << "Estimated Transform: " << e_sim << endl
       << "Euclidian norm     : " << (matrix_4x4d(m_sim) - matrix_4x4d(e_sim)).frobenius_norm() << endl;

  TEST_NEAR("Crafted and estimated similarity transforms match",
            (matrix_4x4d(m_sim) - matrix_4x4d(e_sim)).frobenius_norm(),
            0.0, 1e-12);
}


IMPLEMENT_TEST(reprojection_100pts_noisy)
{
  // Process:
  //  1) generated points
  //  2) make up similarity transform and transform points
  //  3) generate similarity transform via estimation between two points sets
  //  4) check difference between crafted and estimated similarity transforms

  using namespace std;
  using namespace maptk;

  cerr << "Constructing 100 original, random points (std dev: 1.0)" << endl;
  vector<vector_3d> original_points;
  for (int i=0; i < 100; ++i)
  {
    original_points.push_back(testing::random_point3d(1.0));
  }

  cerr << "Constructing crafted similarity transformation" << endl;
  similarity_d m_sim(5.623,
                     rotation_d(vector_3d(-1.4, 0.23, 1.7)),
                     vector_3d(2.24, 1.51, 4.23));

  cerr << "Transforming original points by crafted transformation" << endl;
  vector<vector_3d> transformed_points;
  BOOST_FOREACH(vector_3d o_vec, original_points)
  {
    transformed_points.push_back((m_sim * o_vec) + testing::random_point3d(0.01));
  }

  cerr << "Estimating similarity transformation between point sets" << endl;
  vxl::estimate_similarity_transform est_ST;
  similarity_d e_sim = est_ST.estimate_transform(original_points,
                                                 transformed_points);

  cerr << "Original Transform : " << m_sim << endl
       << "Estimated Transform: " << e_sim << endl
       << "Euclidian norm     : " << (matrix_4x4d(m_sim) - matrix_4x4d(e_sim)).frobenius_norm() << endl;

  TEST_NEAR("Crafted and estimated similarity transforms match",
            (matrix_4x4d(m_sim) - matrix_4x4d(e_sim)).frobenius_norm(),
            0.0, 0.01);
}
