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
 * \brief Homography and derived class functionality regression tests
 */

#include <test_common.h>

#include <maptk/exceptions/math.h>
#include <maptk/homography.h>
#include <maptk/homography_f2f.h>
#include <maptk/logging_macros.h>

#include <Eigen/LU>


#define TEST_ARGS ()
DECLARE_TEST_MAP();


#define TEST_LOG( msg ) \
  LOG_INFO( "test::core::homography", msg )


int main(int argc, char* argv[])
{
  // only expecting the test name
  CHECK_ARGS(1);

  testname_t const testname = argv[1];

  RUN_TEST( testname );
}


namespace // anonymous
{


// Test invert function Eigen::Matrix derived classes
template< typename T >
static bool test_numeric_invertibility()
{
  maptk::homography_<T> invertible,
                        expected_result,
                        noninvertable,
                        ni_result;

  invertible.get_matrix() << 1, 1, 2,
                             3, 4, 5,
                             6, 7, 9;
  expected_result.get_matrix() << -0.5, -2.5,  1.5,
                                  -1.5,  1.5, -0.5,
                                   1.5,  0.5, -0.5;

  maptk::homography_<T> h_inverse( *invertible.inverse() );
  if( ! h_inverse.get_matrix().isApprox( expected_result.get_matrix() ) )
  {
    return false;
  }

  noninvertable.get_matrix() << 1, 2, 3,
                                4, 5, 6,
                                7, 8, 9;
  bool is_invertible;
  noninvertable.get_matrix().computeInverseWithCheck( ni_result.get_matrix(), is_invertible );
  return is_invertible == false;
}


// Test mapping a point for a homography/point data type
template <typename T>
static void test_point_map()
{
  Eigen::Matrix<T,2,1> test_p(1, 1);
  T e = Eigen::NumTraits<T>::dummy_precision();

  {
  // Where [2,2] = 0
  maptk::homography_<T> h_0;
  h_0.get_matrix() << 1.0, 0.0, 1.0,
                      0.0, 1.0, 1.0,
                      0.0, 0.0, 0.0;
  EXPECT_EXCEPTION(
    maptk::point_maps_to_infinity,
    h_0.map_point( test_p ),
    "Applying point to matrix with 0-value lower-right corner"
    );
  }

  {
  // Where [2,2] = e, which is the approximately-zero threshold
  maptk::homography_<T> h_e;
  h_e.get_matrix() << 1.0, 0.0, 1.0,
                      0.0, 1.0, 1.0,
                      0.0, 0.0,  e ;
  EXPECT_EXCEPTION(
    maptk::point_maps_to_infinity,
    h_e.map_point( test_p ),
    "Applying point to matrix with e-value lower-right corner"
    );
  }

  {
  // Matrix: [ 1 0 1  ]
  //         [ 0 1 1  ]
  //         [ 0 0 .5 ]
  // Where [2,2] = 0.5, which should be valid.
  maptk::homography_<T> h_half;
  h_half.get_matrix() << 1.0, 0.0, 1.0,
                         0.0, 1.0, 1.0,
                         0.0, 0.0, 0.5;
  Eigen::Matrix<T,2,1> r = h_half.map_point( test_p );
  TEST_NEAR( "test_point_map::0", r[0], 4, e );
  TEST_NEAR( "test_point_map::1", r[1], 4, e );
  }
}


} // end anonymous namespace


IMPLEMENT_TEST(homography_inversion)
{
  TEST_LOG( "Testing basic homography functions" );

  TEST_EQUAL( "homography::inverse-double",
              test_numeric_invertibility<double>(),
              true );
  TEST_EQUAL( "homography::inverse-float",
              test_numeric_invertibility<float>(),
              true );
}


IMPLEMENT_TEST(f2f_homography_inversion)
{
  TEST_LOG( "Testing Frame-to-frame homography functions" );

  // testing from and to frame swapping during inversion
  maptk::matrix_3x3d i( maptk::matrix_3x3d::Identity() );
  maptk::f2f_homography h( i, 0, 10 ),
                        h_inv(0);
  h_inv = h.inverse();

  TEST_EQUAL( "f2f-homog ref frame inversion - From slot",
              h_inv.from_id(), 10 );
  TEST_EQUAL( "f2f-homog ref frame inversion - To slot",
              h_inv.to_id(), 0 );
}


IMPLEMENT_TEST(map_point)
{
  TEST_LOG( "Testing mapping of a point to against a homography "
            "transformation" );

  // Identity transformation
  maptk::homography_<float> h_f;
  maptk::homography_<double> h_d;

  typedef Eigen::Matrix<float,2,1> pf_t;
  typedef Eigen::Matrix<double,2,1> pd_t;

  pf_t p_f( 2.2, 3.3 );
  pd_t p_d( 5.5, 6.6 );

  // Float Homography
  TEST_LOG( "Calling float H with float P" );
  pf_t rf_hf = h_f.map_point(p_f);
  TEST_EQUAL( "map_point::float_H::float_p::return_float",
              rf_hf, p_f );

  // Double homography
  TEST_LOG( "Calling double H with double P" );
  pd_t rd_hd = h_d.map_point(p_d);
  TEST_EQUAL( "map_point::double_h::double_p",
              rd_hd, p_d );
}


IMPLEMENT_TEST(map_point_zero_div)
{
  TEST_LOG( "Testing zero-division protection when mapping points with a "
            "homography" );
  test_point_map<float>();
  test_point_map<double>();
}
