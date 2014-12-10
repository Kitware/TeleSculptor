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
 * \brief Header for \link maptk::matrix_ matrix_<M,N,T> \endlink class
 */

#ifndef MAPTK_MATRIX_H_
#define MAPTK_MATRIX_H_

#include <maptk/config.h>

#include <iostream>
#include <cstring>
#include <cassert>

#include <boost/integer/static_min_max.hpp>
#include <boost/static_assert.hpp>

#include "vector.h"
#include "vector_cmath.h"
#include "exceptions.h"

#include <Eigen/Core>


namespace maptk
{

/// A representation of a matrix
template <unsigned M, unsigned N, typename T>
class MAPTK_LIB_EXPORT matrix_
{
public:
  /// a compile time constant defined to be min(M,N)
  static unsigned const min_dim = boost::static_unsigned_min<M,N>::value;
  /// a compile time constant defined to be max(M,N)
  static unsigned const max_dim = boost::static_unsigned_max<M,N>::value;
  /// a compile time constant defined to be M*N
  static unsigned const num_elems = M*N;
  /// typedef for vector cmath (i.e. treat the matrix as a vector)
  typedef vector_cmath_<num_elems,T> cmath;


  /// Constructor - does not initialize
  matrix_<M,N,T>() {}

  /// Copy Constructor
  matrix_<M,N,T>(const matrix_<M,N,T>& other)
  {
    memcpy( data_[0], other.data_[0], M*N*sizeof(T) );
  }

  /// Copy Constructor from another type
  template <typename U>
  explicit matrix_<M,N,T>(const matrix_<M,N,U>& other)
  {
    const U* in = other.data();
    T* out = this->data_;
    for(unsigned i=0; i<num_elems; ++i, ++in, ++out)
    {
      *out = static_cast<T>(*in);
    }
  }

  /// Constructor - from block of data (row-wise)
  explicit matrix_<M,N,T>(const T* data)
  {
    memcpy( data_[0], data, M*N*sizeof(T) );
  }

  /// Constructor - fill with a constant value
  explicit matrix_<M,N,T>(const T& value) { this->fill(value); }

  /// Assignment operator
  matrix_<M,N,T>& operator=(const matrix_<M,N,T>& other)
  {
    memcpy( data_[0], other.data_[0], M*N*sizeof(T) );
    return *this;
  }

  /// Return the i-th row
  T* operator[](unsigned int i) { return data_[i]; }

  /// Return the i-th row (const)
  T const* operator[](unsigned int i) const { return data_[i]; }

  /// Return the i-th row, j-th column
  T& operator()(unsigned int i, unsigned int j)
  {
    assert(i<M);
    assert(j<N);
    return data_[i][j];
  }

  /// Return the i-th row (const)
  const T& operator()(unsigned int i, unsigned int j) const
  {
    assert(i<M);
    assert(j<N);
    return data_[i][j];
  }

  /// Return a pointer to the contiguous block of memory
  T* data() { return data_[0]; }

  /// Return a pointer to the contiguous block of memory
  T const* data() const { return data_[0]; }

  /// Fill the matrix with this value
  matrix_<M,N,T>& fill(const T& value);

  /// Fill the diagonal with this value
  matrix_<M,N,T>& fill_diagonal(const T& value);

  /// Set the diagonal to this vector
  matrix_<M,N,T>& set_diagonal(const vector_<min_dim,T>& diag);

  /// Set the matrix to the identity matrix
  /**
   * Extra rows or columns of a non-square matrix are set to zero
   */
  matrix_<M,N,T>& set_identity();

  /// Return the transpose of this matrix
  matrix_<N,M,T> transpose() const;

  /// Update a sub-block of this matrix located at (top, left)
  template <unsigned A, unsigned B>
  matrix_<M,N,T>& update(const matrix_<A,B,T>& m,
                         unsigned top=0, unsigned left=0)
  {
    BOOST_STATIC_ASSERT(A<=N);
    BOOST_STATIC_ASSERT(B<=M);
    assert(top + A <= M);
    assert(left + B <= N);
    for (unsigned int i=0; i<A; ++i)
    {
      for (unsigned int j=0; j<B; ++j)
      {
        this->data_[i+top][j+left] = m(i, j);
      }
    }
    return *this;
  }

  /// Extract a sub-block of this matrix located at (top, left)
  template <unsigned A, unsigned B>
  void extract(matrix_<A,B,T>& m,
               unsigned top=0, unsigned left=0) const
  {
    BOOST_STATIC_ASSERT(A<=N);
    BOOST_STATIC_ASSERT(B<=M);
    assert(top + A <= M);
    assert(left + B <= N);
    for (unsigned int i=0; i<A; ++i)
    {
      for (unsigned int j=0; j<B; ++j)
      {
        m(i,j) = this->data_[i+top][j+left];
      }
    }
  }

  /// Update a row of the matrix with the values in a vector
  matrix_<M,N,T>& set_row(unsigned row, const vector_<N,T>& v)
  {
    for (unsigned int j=0; j<N; ++j)
    {
      this->data_[row][j] = v[j];
    }
    return *this;
  }

  /// Update a column of the matrix with the values in a vector
  matrix_<M,N,T>& set_column(unsigned col, const vector_<M,T>& v)
  {
    for (unsigned int i=0; i<M; ++i)
    {
      this->data_[i][col] = v[i];
    }
    return *this;
  }

  /// Access a row of the matrix
  vector_<N,T> row(unsigned r) const
  {
    return vector_<N,T>(this->data_[r]);
  }

  /// Access a column of the matrix
  vector_<M,T> column(unsigned c) const
  {
    vector_<M,T> v;
    for (unsigned int i=0; i<M; ++i)
    {
      v[i] = this->data_[i][c];
    }
    return v;
  }

  /// Return the Frobenius norm of the matrix (sqrt of sum of squares)
  T frobenius_norm() const { return cmath::l2_norm( data_[0] ); }

  /// Add a scalar in place
  matrix_<M,N,T>& operator+=( T s ) { cmath::add( data_[0], s, data_[0] ); return *this; }

  /// Subract a scalr in place
  matrix_<M,N,T>& operator-=( T s ) { cmath::sub( data_[0], s, data_[0] ); return *this; }

  /// Multiply a scalar in place
  matrix_<M,N,T>& operator*=( T s ) { cmath::mul( data_[0], s, data_[0] ); return *this; }

  /// Divide by a scalar in place
  matrix_<M,N,T>& operator/=( T s ) { cmath::div( data_[0], s, data_[0] ); return *this; }

  /// Add a matrix in place
  matrix_<M,N,T>& operator+=( const matrix_<M,N,T>& m )
  {
    cmath::add( data_[0], m.data_[0], data_[0] );
    return *this;
  }

  /// Subract a matrix in place
  matrix_<M,N,T>& operator-=( const matrix_<M,N,T>& m )
  {
    cmath::sub( data_[0], m.data_[0], data_[0] );
    return *this;
  }

  /// Negate operator
  matrix_<M,N,T> operator-() const
  {
    matrix_<M,N,T> result;
    cmath::sub( T(0), data_[0], result.data_[0] );
    return result;
  }


protected:
  /// matrix data
  T data_[M][N];
};


/// \cond DoxygenSuppress
typedef Eigen::Matrix<double, 2, 2> matrix_2x2d;
typedef Eigen::Matrix<float, 2, 2>  matrix_2x2f;
typedef Eigen::Matrix<double, 2, 3> matrix_2x3d;
typedef Eigen::Matrix<float, 2, 3>  matrix_2x3f;
typedef Eigen::Matrix<double, 3, 2> matrix_3x2d;
typedef Eigen::Matrix<float, 3, 2>  matrix_3x2f;
typedef Eigen::Matrix<double, 3, 3> matrix_3x3d;
typedef Eigen::Matrix<float, 3,3>   matrix_3x3f;
typedef Eigen::Matrix<double, 3, 4> matrix_3x4d;
typedef Eigen::Matrix<float, 3, 4>  matrix_3x4f;
typedef Eigen::Matrix<double, 4, 3> matrix_4x3d;
typedef Eigen::Matrix<float, 4, 3>  matrix_4x3f;
typedef Eigen::Matrix<double, 4, 4> matrix_4x4d;
typedef Eigen::Matrix<float, 4, 4>  matrix_4x4f;
/// \endcond

// --- Matrix-scalar operators ----------------------------------------

/// Matrix-scalar addtion operator
/**
 * \relatesalso matrix_
 * \param m a matrix
 * \param s a scalar
 */
template <unsigned M, unsigned N, typename T>
inline matrix_<M,N,T> operator+( const matrix_<M,N,T>& m, const T& s )
{
  matrix_<M,N,T> r;
  vector_cmath_<M*N,T>::add( m.data(), s, r.data() );
  return r;
}

/// Scalar-matrix addition operator
/**
 * \relatesalso matrix_
 * \param m a matrix
 * \param s a scalar
 */
template <unsigned M, unsigned N, typename T>
inline matrix_<M,N,T> operator+( const T& s, const matrix_<M,N,T>& m )
{
  matrix_<M,N,T> r;
  vector_cmath_<M*N,T>::add( m.data(), s, r.data() );
  return r;
}

/// Matrix-scalar subraction operator
/**
 * \relatesalso matrix_
 * \param m a matrix
 * \param s a scalar
 */
template <unsigned M, unsigned N, typename T>
inline matrix_<M,N,T> operator-( const matrix_<M,N,T>& m, const T& s )
{
  matrix_<M,N,T> r;
  vector_cmath_<M*N,T>::sub( m.data(), s, r.data() );
  return r;
}

/// Scalar-matrix subraction operator
/**
 * \relatesalso matrix_
 * \param m a matrix
 * \param s a scalar
 */
template <unsigned M, unsigned N, typename T>
inline matrix_<M,N,T> operator-( const T& s, const matrix_<M,N,T>& m )
{
  matrix_<M,N,T> r;
  vector_cmath_<M*N,T>::sub( s, m.data(), r.data() );
  return r;
}

/// Scalar post-multiplcation operator
/**
 * \relatesalso matrix_
 * \param m a matrix
 * \param s a scalar
 */
template <unsigned M, unsigned N, typename T>
inline matrix_<M,N,T> operator*( const matrix_<M,N,T>& m, const T& s )
{
  matrix_<M,N,T> r;
  vector_cmath_<M*N,T>::mul( m.data(), s, r.data() );
  return r;
}

/// Scalar pre-multiplication operator
/**
 * \relatesalso matrix_
 * \param m a matrix
 * \param s a scalar
 */
template <unsigned M, unsigned N, typename T>
inline matrix_<M,N,T> operator*( const T& s, const matrix_<M,N,T>& m )
{
  matrix_<M,N,T> r;
  vector_cmath_<M*N,T>::mul( m.data(), s, r.data() );
  return r;
}

/// Scalar division operator
/**
 * \relatesalso matrix_
 * \param m a matrix
 * \param s a scalar
 */
template <unsigned M, unsigned N, typename T>
inline matrix_<M,N,T> operator/( const matrix_<M,N,T>& m, const T& s )
{
  matrix_<M,N,T> r;
  vector_cmath_<M*N,T>::div( m.data(), s, r.data() );
  return r;
}


// --- Matrix-matrix operators ----------------------------------------

/// Addition operator
/**
 * \relatesalso matrix_
 * \param a a matrix
 * \param b another matrix
 */
template <unsigned M, unsigned N, typename T>
inline matrix_<M,N,T> operator+( const matrix_<M,N,T>& a, const matrix_<M,N,T>& b )
{
  matrix_<M,N,T> r;
  vector_cmath_<M*N,T>::add( a.data(), b.data(), r.data() );
  return r;
}

/// Subraction operator
/**
 * \relatesalso matrix_
 * \param a a matrix
 * \param b another matrix
 */
template <unsigned M, unsigned N, typename T>
inline matrix_<M,N,T> operator-( const matrix_<M,N,T>& a, const matrix_<M,N,T>& b )
{
  matrix_<M,N,T> r;
  vector_cmath_<M*N,T>::sub( a.data(), b.data(), r.data() );
  return r;
}

/// Element-wise product
/**
 * \relatesalso matrix_
 * \param a a matrix
 * \param b another matrix
 */
template <unsigned M, unsigned N, typename T>
inline matrix_<M,N,T> element_product( const matrix_<M,N,T>& a, const matrix_<M,N,T>& b )
{
  matrix_<M,N,T> r;
  vector_cmath_<M*N,T>::mul( a.data(), b.data(), r.data() );
  return r;
}

/// Element-wise quotient
/**
 * \relatesalso matrix_
 * \param a a matrix
 * \param b another matrix
 */
template <unsigned M, unsigned N, typename T>
inline matrix_<M,N,T> element_quotient( const matrix_<M,N,T>& a, const matrix_<M,N,T>& b )
{
  matrix_<M,N,T> r;
  vector_cmath_<M*N,T>::div( a.data(), b.data(), r.data() );
  return r;
}

/// Equality operator
/**
 * \relatesalso matrix_
 * \param a a matrix
 * \param b another matrix
 */
template <unsigned M, unsigned N, typename T>
inline bool operator==( const matrix_<M,N,T>& a, const matrix_<M,N,T>& b )
{
  return vector_cmath_<M*N,T>::eq(a.data(), b.data());
}

/// Inequality operator
/**
 * \relatesalso matrix_
 * \param a a matrix
 * \param b another matrix
 */
template <unsigned M, unsigned N, typename T>
inline bool operator!=( const matrix_<M,N,T>& a, const matrix_<M,N,T>& b )
{
  return ! vector_cmath_<M*N,T>::eq(a.data(), b.data());
}

// --- Matrix and vector multiplication -----------------------------------

/// Multiply matrix_ (M x N) and vector_ (N)
/**
 * \relatesalso vector_
 * \relatesalso matrix_
 * \param a a matrix
 * \param b a vector
 */
template <unsigned M, unsigned N, typename T>
inline
vector_<M,T> operator*(const matrix_<M,N,T>& a, const vector_<N,T>& b)
{
  vector_<M,T> out;
  for (unsigned i = 0; i < M; ++i)
  {
    T accum = a(i,0) * b[0];
    for (unsigned k = 1; k < N; ++k)
      accum += a(i,k) * b[k];
    out[i] = accum;
  }
  return out;
}

/// Multiply vector_ (M) and matrix_ (M x N)
/**
 * \relatesalso vector_
 * \relatesalso matrix_
 * \param a a vector
 * \param b a matrix
 */
template <unsigned M, unsigned N, typename T>
inline
vector_<N,T> operator*(const vector_<M,T>& a, const matrix_<M,N,T>& b)
{
  vector_<N,T> out;
  for (unsigned i = 0; i < N; ++i)
  {
    T accum = a[0] * b(0,i);
    for (unsigned k = 1; k < M; ++k)
      accum += a[k] * b(k,i);
    out[i] = accum;
  }
  return out;
}

/// Multiply two matrix_ (M x N) times (N x O)
/**
 * \relatesalso matrix_
 * \param a a matrix
 * \param b another matrix
 */
template <unsigned M, unsigned N, unsigned O, typename T>
inline
matrix_<M,O,T> operator*(const matrix_<M,N,T>& a, const matrix_<N,O,T>& b)
{
  matrix_<M,O,T> out;
  for (unsigned i = 0; i < M; ++i)
    for (unsigned j = 0; j < O; ++j)
    {
      T accum = a(i,0) * b(0,j);
      for (unsigned k = 1; k < N; ++k)
        accum += a(i,k) * b(k,j);
      out(i,j) = accum;
    }
  return out;
}

/// Compute the determinant of a 2x2 square matrix
template <typename T>
inline
T determinant(const matrix_<2,2,T>& m)
{
  const T* d = m.data();
  return d[0]*d[3] - d[1]*d[2];
}

/// Compute the determinant of a 3x3 square matrix
template <typename T>
inline
T determinant(const matrix_<3,3,T>& m)
{
  const T* d = m.data();
  return d[0]*(d[4]*d[8] - d[5]*d[7])
       + d[1]*(d[5]*d[6] - d[3]*d[8])
       + d[2]*(d[3]*d[7] - d[4]*d[6]);
}

/// Compute the inverse of a 2x2 square matrix
template <typename T>
matrix_<2,2,T> inverse(const matrix_<2,2,T>& m)
{
  T det = determinant(m);
  if (det==0)
  {
    throw non_invertible_matrix();
  }
  det = T(1)/det;
  T d[4];
  d[0] = m(1,1)*det;
  d[1] = -m(0,1)*det;
  d[2] = -m(1,0)*det;
  d[3] = m(0,0)*det;
  return matrix_<2,2,T>(d);
}

/// Compute the inverse of a 3x3 square matrix
template <typename T>
matrix_<3,3,T> inverse(const matrix_<3,3,T>& m)
{
  T det = determinant(m);
  if (det==0)
  {
    throw non_invertible_matrix();
  }
  det = T(1)/det;
  T d[9];
  d[0] = (m(1,1)*m(2,2)-m(1,2)*m(2,1))*det;
  d[1] = (m(2,1)*m(0,2)-m(2,2)*m(0,1))*det;
  d[2] = (m(0,1)*m(1,2)-m(0,2)*m(1,1))*det;
  d[3] = (m(1,2)*m(2,0)-m(1,0)*m(2,2))*det;
  d[4] = (m(0,0)*m(2,2)-m(0,2)*m(2,0))*det;
  d[5] = (m(1,0)*m(0,2)-m(1,2)*m(0,0))*det;
  d[6] = (m(1,0)*m(2,1)-m(1,1)*m(2,0))*det;
  d[7] = (m(0,1)*m(2,0)-m(0,0)*m(2,1))*det;
  d[8] = (m(0,0)*m(1,1)-m(0,1)*m(1,0))*det;
  return matrix_<3,3,T>(d);
}

/// Compute the cross_product 3x3 matrix from a 3D vector
/**
 * Produces a matrix such that
 * \code
 *   cross_product(v1) * v2 == cross_product(v1, v2)
 * \endcode
 */
template <typename T>
matrix_<3,3,T> cross_product(const vector_<3,T>& v)
{
  matrix_<3,3,T> x;
  x(0,0) = x(1,1) = x(2,2) = T(0);
  x(0,1) = -v[2];
  x(1,0) =  v[2];
  x(2,0) = -v[1];
  x(0,2) =  v[1];
  x(1,2) = -v[0];
  x(2,1) =  v[0];
  return x;
}


/// output stream operator for a matrix
/**
 * \param s an output stream
 * \param m a matrix to stream
 */
template <unsigned M, unsigned N, typename T>
MAPTK_LIB_EXPORT std::ostream&  operator<<(std::ostream& s, const matrix_<M,N,T>& m);

/// input stream operator for a matrix
/**
 * \param s an input stream
 * \param m a matrix to stream into
 */
template <unsigned M, unsigned N, typename T>
MAPTK_LIB_EXPORT std::istream&  operator>>(std::istream& s, matrix_<M,N,T>& m);


} // end namespace maptk


#endif // MAPTK_MATRIX_H_
