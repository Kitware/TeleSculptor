/*ckwg +5
 * Copyright 2011-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief test support functions involving generic math
 */

#ifndef MAPTK_TEST_MATH_H_
#define MAPTK_TEST_MATH_H_

#include <cmath>

#include <maptk/core/matrix.h>
#include <maptk/core/vector.h>


namespace maptk
{

namespace testing
{


/// Near comparison function for vectors
/**
 * Drop-in compatible with TEST_NEAR. Just need to include this header.
 */
template <unsigned N, typename T>
bool is_almost(vector_<N,T> const& a, vector_<N,T> const& b,
               double const& epsilon)
{
  for (unsigned i=0; i<N; ++i)
  {
    if (fabs(a[i] - b[i]) > epsilon)
    {
      return false;
    }
  }
  return true;
}


/// Near comparison function for vectors
/**
 * Drop-in compatible with TEST_NEAR. Just need to include this header.
 */
template <unsigned M, unsigned N, typename T>
bool is_almost(matrix_<M,N,T> const& a, matrix_<M,N,T> const& b,
               double const& epsilon)
{
  for (unsigned i=0; i<M; ++i)
  {
    for (unsigned j=0; j<N; ++j)
    {
      if (fabs(a(i,j) - b(i,j)) > epsilon)
      {
        return false;
      }
    }
  }
  return true;
}


}

}


#endif // MAPTK_TEST_MATH_H_
