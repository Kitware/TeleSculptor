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
 * \brief test core essential matrix class
 */

#include <test_common.h>
#include <test_math.h>

#include <iostream>
#include <vector>

#include <maptk/essential_matrix.h>

#include <Eigen/SVD>
#include <boost/foreach.hpp>
#include <boost/math/constants/constants.hpp>


#define TEST_ARGS ()

DECLARE_TEST_MAP();

int
main(int argc, char* argv[])
{
  CHECK_ARGS(1);

  testname_t const testname = argv[1];

  RUN_TEST(testname);
}


IMPLEMENT_TEST(matrix_properties)
{
  using namespace maptk;
  rotation_d rot(vector_3d(1.0, 2.0, 3.0));
  vector_3d t(-1.0, 1.0, 4.0);

  essential_matrix_d em(rot, t);
  matrix_3x3d mat = em.matrix();

  Eigen::JacobiSVD<matrix_3x3d> svd(mat, Eigen::ComputeFullV |
                                         Eigen::ComputeFullU);
  TEST_NEAR("Singular values should be [1 1 0]",
            svd.singularValues(), vector_3d(1.0, 1.0, 0.0), 1e-14);

  TEST_NEAR("translation should be unit length",
            em.translation().norm(), 1.0, 1e-14);

  const matrix_3x3d W = (matrix_3x3d() << 0.0, -1.0, 0.0,
                                          1.0,  0.0, 0.0,
                                          0.0,  0.0, 1.0).finished();
  const matrix_3x3d& U = svd.matrixU();
  const matrix_3x3d& V = svd.matrixV();
  vector_3d t_extracted = U.col(2);

  vector_3d t_norm = t.normalized();
  if(!testing::is_almost(t_norm, t_extracted, 1e-14) &&
     !testing::is_almost(t_norm, (-t_extracted).eval(), 1e-14) )
  {
    TEST_ERROR("extract translation should match original up to a sign\n"
               "input: " << t.normalized().transpose() << "\n"
               "got:   " << t_extracted.transpose() );
  }

  matrix_3x3d R1_extracted = U*W*V.transpose();
  matrix_3x3d R2_extracted = U*W.transpose()*V.transpose();
  if( R1_extracted.determinant() < 0.0 )
  {
    R1_extracted *= -1.0;
  }
  if( R2_extracted.determinant() < 0.0 )
  {
    R2_extracted *= -1.0;
  }

  if(!testing::is_almost(matrix_3x3d(rot), R1_extracted, 1e-14) &&
     !testing::is_almost(matrix_3x3d(rot), R2_extracted, 1e-14) )
  {
    TEST_ERROR("extracted rotation should match input or twisted pair\n"
               "input:\n" << matrix_3x3d(rot) << "\n"
               "got (v1):\n" << R1_extracted << "\n"
               "got (v2):\n" << R2_extracted << "\n");
  }
}


IMPLEMENT_TEST(twisted_pair)
{
  using namespace maptk;
  const double epsilon = 1e-14;
  rotation_d rot(vector_3d(1.0, 2.0, 3.0));
  vector_3d t(-1.0, 1.0, 4.0);

  essential_matrix_d em(rot, t);

  // any combination of these should be an equivalent essential matrix
  rotation_d R1 = em.rotation();
  rotation_d R2 = em.twisted_rotation();
  vector_3d t1 = em.translation();
  vector_3d t2 = -t1;

  rotation_d rot_t_180(boost::math::constants::pi<double>(),
                       t.normalized().eval());
  TEST_NEAR("twisted pair rotation should be 180 degree rotation around t",
            matrix_3x3d(R2), matrix_3x3d(rot_t_180 * R1), epsilon);

  essential_matrix_d em1(R1, t1), em2(R1, t2), em3(R2, t1), em4(R2, t2);
  matrix_3x3d M1(em1.matrix()), M2(em2.matrix()),
              M3(em3.matrix()), M4(em4.matrix());
  matrix_3x3d M(em.matrix());

  // flip sign on matrices with mismatched signs
  if( (M(0,0) > 0.0) != (M1(0,0) > 0.0) )
  {
    M1 *= -1.0;
  }
  TEST_NEAR("Possible factorization 1 matches source",
            M, M1, 1e-14);

  // flip sign on matrices with mismatched signs
  if( (M(0,0) > 0.0) != (M2(0,0) > 0.0) )
  {
    M2 *= -1.0;
  }
  TEST_NEAR("Possible factorization 2 matches source",
            M, M2, 1e-14);

  // flip sign on matrices with mismatched signs
  if( (M(0,0) > 0.0) != (M3(0,0) > 0.0) )
  {
    M3 *= -1.0;
  }
  TEST_NEAR("Possible factorization 3 matches source",
            M, M3, 1e-14);

  // flip sign on matrices with mismatched signs
  if( (M(0,0) > 0.0) != (M4(0,0) > 0.0) )
  {
    M4 *= -1.0;
  }
  TEST_NEAR("Possible factorization 4 matches source",
            M, M4, 1e-14);
}
