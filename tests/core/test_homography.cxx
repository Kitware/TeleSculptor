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

#include <maptk/homography.h>
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
bool test_invertibility( T base_instance )
{
  // Create a few copies of the base instance for manipulation
  T invertible(base_instance),
    expected_result(base_instance),
    noninvertable(base_instance),
    ni_result(base_instance);

  invertible << 1, 1, 2,
                3, 4, 5,
                6, 7, 9;
  expected_result << -0.5, -2.5,  1.5,
                     -1.5,  1.5, -0.5,
                      1.5,  0.5, -0.5;
  if( ! invertible.inverse().isApprox( expected_result ) )
  {
    return false;
  }

  noninvertable << 1, 2, 3,
                   4, 5, 6,
                   7, 8, 9;
  bool is_invertible;
  noninvertable.computeInverseWithCheck( ni_result, is_invertible );
  return is_invertible == false;
}

} // end anonymous namespace


IMPLEMENT_TEST(homography)
{
  TEST_LOG( "Testing basic homography functions" );

  TEST_EQUAL( "homography::inverse",
              test_invertibility< maptk::homography >( maptk::homography() ),
              true );
}


IMPLEMENT_TEST(f2f_homography)
{
  TEST_LOG( "Testing Frame-to-frame homography functions" );

  TEST_EQUAL( "f2f_homography::inverse",
              test_invertibility< maptk::f2f_homography >( maptk::f2f_homography(0) ),
              true );
}


IMPLEMENT_TEST(f2w_homography)
{
  TEST_LOG( "Testing Frame-to-world homography functions" );

  TEST_EQUAL( "f2w_homography::inverse",
              test_invertibility< maptk::f2w_homography >( maptk::f2w_homography(0) ),
              true );
}
