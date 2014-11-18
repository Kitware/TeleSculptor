/*ckwg +29
 * Copyright 2013-2014 by Kitware, Inc.
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
