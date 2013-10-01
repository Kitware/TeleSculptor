/*ckwg +5
 * Copyright 2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_MATRIX_H_
#define MAPTK_MATRIX_H_

#include <iostream>
#include <cassert>

namespace maptk
{

/// A representation of a matrix
template <unsigned M, unsigned N, typename T>
class matrix_
{
public:
  /// Constructor - does not initialize
  matrix_<M,N,T>() {}

  /// Copy Constructor
  matrix_<M,N,T>(const matrix_<M,N,T>& other)
  {
    memcpy( data_[0], other.data_[0], M*N*sizeof(T) );
  }

  /// Constructor - from block of data (row-wise)
  explicit matrix_<M,N,T>(const T* data)
  {
    memcpy( data_[0], data, M*N*sizeof(T) );
  }

  /// Constructor - fill with a constant value
  explicit matrix_<M,N,T>(const T& value)
  {
    T* p = data_[0];
    unsigned int n = M*N;
    while(n--)
    {
      *p++ = value;
    }
  }

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

protected:
  T data_[M][N];
};


typedef matrix_<2,2,double> matrix_2x2d;
typedef matrix_<2,2,float> matrix_2x2f;
typedef matrix_<3,3,double> matrix_3x3d;
typedef matrix_<3,3,float> matrix_3x3f;
typedef matrix_<3,4,double> matrix_3x4d;
typedef matrix_<3,4,float> matrix_3x4f;

/// output stream operator for a vector
template <unsigned M, unsigned N, typename T>
std::ostream&  operator<<(std::ostream& s, const matrix_<M,N,T>& v);

/// input stream operator for a vector
template <unsigned M, unsigned N, typename T>
std::istream&  operator>>(std::istream& s, matrix_<M,N,T>& v);


} // end namespace maptk


#endif // MAPTK_MATRIX_H_
