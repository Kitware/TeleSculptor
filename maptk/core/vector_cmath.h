/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief vector math templated functions
 */

#ifndef MAPTK_VECTOR_CMATH_H_
#define MAPTK_VECTOR_CMATH_H_

#include <cmath>

namespace maptk
{

/// C-style vector math operatorations
/**
 * This struct is intended to provided efficient helper
 * functions for basic vector operations.  These functions
 * are templated over scalar type and dimension.
 * All members of this struct are static
 */
template <unsigned N, typename T>
struct vector_cmath_
{
public:
  /// typedef for the scalar type used
  typedef T scalar;
  /// A compile-time constant for the dimension
  static const unsigned int dim = N;

  /// Add a vector to a vector
  inline static void add( const T* a, const T* b, T* r )
  {
    for ( unsigned int i=0; i < N; ++i,++r,++a,++b )
      *r = *a + *b;
  }

  /// Add a scalar to a vector
  inline static void add( const T* a, T b, T* r )
  {
    for ( unsigned int i=0; i < N; ++i,++r,++a )
      *r = *a + b;
  }

  /// Subract a vector from a vector
  inline static void sub( const T* a, const T* b, T* r )
  {
    for ( unsigned int i=0; i < N; ++i,++r,++a,++b )
      *r = *a - *b;
  }

  /// Subract a scalar from a vector
  inline static void sub( const T* a, T b, T* r )
  {
    for ( unsigned int i=0; i < N; ++i,++r,++a )
      *r = *a - b;
  }

  /// Subtract a vector from a scalar
  inline static void sub( T a, const T* b, T* r )
  {
    for ( unsigned int i=0; i < N; ++i,++r,++b )
      *r = a - *b;
  }

  /// Multiply a vector by a vector element-wise
  inline static void mul( const T* a, const T* b, T* r )
  {
    for ( unsigned int i=0; i < N; ++i,++r,++a,++b )
      *r = *a * *b;
  }

  /// Multiply a vector by a scalar
  inline static void mul( const T* a, T b, T* r )
  {
    for ( unsigned int i=0; i < N; ++i,++r,++a )
      *r = *a * b;
  }

  /// Divide a vector by a vector element-wise
  inline static void div( const T* a, const T* b, T* r )
  {
    for ( unsigned int i=0; i < N; ++i,++r,++a,++b )
      *r = *a / *b;
  }

  /// Divide a vector by a scalar
  inline static void div( const T* a, T b, T* r )
  {
    for ( unsigned int i=0; i < N; ++i,++r,++a )
      *r = *a / b;
  }

  /// Test for vector equality
  inline static bool eq( const T* a, const T* b )
  {
    for ( unsigned int i=0; i < N; ++i,++a,++b )
    {
      if ( *a != *b )
      {
        return false;
      }
    }
    return true;
  }

  /// The squared L2 norm (sum of squares) of a vector
  inline static T l2_norm_squared( const T* a)
  {
    T val(0);
    for ( unsigned int i=0; i < N; ++i,++a )
    {
      val += (*a)*(*a);
    }
    return val;
  }

  /// The L2 norm (square root of sum of squares) of a vector
  inline static T l2_norm( const T* a)
  {
    return std::sqrt(l2_norm_squared(a));
  }

};

} // end namespace maptk


#endif // MAPTK_VECTOR_CMATH_H_
