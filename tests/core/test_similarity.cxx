/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include <test_common.h>

#include <maptk/core/similarity.h>

#define TEST_ARGS ()

DECLARE_TEST_MAP();

int
main(int argc, char* argv[])
{
  CHECK_ARGS(1);

  testname_t const testname = argv[1];

  RUN_TEST(testname);
}


IMPLEMENT_TEST(default_constructor)
{
  using namespace maptk;
  similarity_d sim;
  if (sim.scale() != 1.0 || sim.rotation() != rotation_d() ||
      sim.translation() != vector_3d(0,0,0))
  {
    TEST_ERROR("The default similarity transformation is not the identity");
  }
}


IMPLEMENT_TEST(convert_matrix)
{
  using namespace maptk;
  similarity_d sim;
  matrix_4x4d S(sim);
  matrix_4x4d I = matrix_4x4d().set_identity();
  std::cout << S << I << std::endl;

  if (S != I)
  {
    TEST_ERROR("Default similarity transformation is not identity matrix");
  }

  similarity_d sim1(2.4, rotation_d(vector_3d(0.1, -1.5, 2.0)),
                    vector_3d(1,-2,5));
  matrix_4x4d mat1(sim1);
  similarity_d sim2(mat1);
  matrix_4x4d mat2(sim2);
  std::cout << "sim1: "<< sim1 <<std::endl;
  std::cout << "sim2: "<< sim2 <<std::endl;
  TEST_NEAR("Convert similarity to matrix and back",
            (mat1 - mat2).frobenius_norm(), 0.0, 1e-14);

}


IMPLEMENT_TEST(compose)
{
  using namespace maptk;

  similarity_d sim1(2.4, rotation_d(vector_3d(0.1, -1.5, 2.0)),
                    vector_3d(1,-2,5));
  similarity_d sim2(0.75, rotation_d(vector_3d(-0.5, -0.5, 1.0)),
                    vector_3d(4,6.5,8));

  matrix_4x4d sim_comp_sim = sim1 * sim2;
  matrix_4x4d mat_comp_sim = matrix_4x4d(sim1) * matrix_4x4d(sim2);
  std::cout << "similarity composition: "<<sim_comp_sim<<std::endl;
  std::cout << "matrix composition: "<<mat_comp_sim<<std::endl;

  TEST_NEAR("Matrix multiplication matches similarity composition",
            (sim_comp_sim - mat_comp_sim).frobenius_norm(),
            0.0, 1e-14);
}


IMPLEMENT_TEST(inverse)
{
  using namespace maptk;

  TEST_EQUAL("Inverse of identity is identity",
             similarity_d(), similarity_d().inverse());

  similarity_d sim1(2.4, rotation_d(vector_3d(0.1, -1.5, 2.0)),
                    vector_3d(1,-2,5));
  similarity_d I = sim1 * sim1.inverse();

  TEST_NEAR("Similarity composed with inverse does not have unit scale",
            I.scale(), 1.0, 1e-14);
  TEST_NEAR("Similarity composed with inverse does not have zero rotation",
            I.rotation().angle(), 0.0, 1e-14);
  TEST_NEAR("Similarity composed with inverse does not have zero translation",
            I.translation().magnitude(), 0.0, 1e-14);
}
