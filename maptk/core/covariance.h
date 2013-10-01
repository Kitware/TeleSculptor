/*ckwg +5
 * Copyright 2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_COVARIANCE_H_
#define MAPTK_COVARIANCE_H_

#include <iostream>
#include "matrix.h"

namespace maptk
{

/// A representation of covariance of a measurement
template <unsigned N, typename T>
class covariance_
{
public:
  /// Number of unique values in a NxN symmetric matrix
  static const unsigned int data_size = N*(N+1)/2;

  /// Default Constructor
  covariance_<N,T>() {}

  /// Constructor - initialize to identity matrix times a scalar
  explicit covariance_<N,T>(const T& value)
  {
    unsigned int n=0;
    for( unsigned int j=0; j<N; ++j)
    {
      for( unsigned int i=0; i<j; ++i)
      {
        data_[n++] = T(0);
      }
      data_[n++] = value;
    }
  }

  /// Constructor - from a matrix
  /// averages off diagonal elements to enforce symmetry
  explicit covariance_<N,T>(const matrix_<N,N,T>& mat)
  {
    unsigned int n=0;
    for( unsigned int j=0; j<N; ++j)
    {
      for( unsigned int i=0; i<j; ++i)
      {
        data_[n++] = (mat(i,j) + mat(j,i)) / 2;
      }
      data_[n++] = mat(j,j);
    }
  }

  operator matrix_<N,N,T>() const
  {
    matrix_<N,N,T> mat;
    unsigned int n=0;
    for( unsigned int j=0; j<N; ++j)
    {
      for( unsigned int i=0; i<j; ++i)
      {
        mat(i,j) = mat(j,i) = data_[n++];
      }
      mat(j,j) = data_[n++];
    }
    return mat;
  }

  /// Return the i-th row, j-th column
  T& operator()(unsigned int i, unsigned int j)
  {
    assert(i<N);
    assert(j<N);
    return data_[vector_index(i,j)];
  }

  /// Return the i-th row, j-th column (const)
  const T& operator()(unsigned int i, unsigned int j) const
  {
    assert(i<N);
    assert(j<N);
    return data_[vector_index(i,j)];
  }

  /// Compute the generalized variance (determinant of covariance)
  T generalized_variance() const;

  /// Access the underlying data
  const T* data() const { return data_; }

protected:
  /// Convert from matrix to vector indices
  unsigned int vector_index(unsigned int i, unsigned int j) const
  {
    return (j>i) ? j*(j+1)/2+i : i*(i+1)/2+j;
  }

  T data_[data_size];
};

typedef covariance_<2,double> covariance_2d;
typedef covariance_<2,float> covariance_2f;
typedef covariance_<3,double> covariance_3d;
typedef covariance_<3,float> covariance_3f;

/// output stream operator for a covariance
template <unsigned N, typename T>
std::ostream&  operator<<(std::ostream& s, const covariance_<N,T>& c);

/// input stream operator for a covariance
template <unsigned N, typename T>
std::istream&  operator>>(std::istream& s, covariance_<N,T>& c);


} // end namespace maptk


#endif // MAPTK_COVARIANCE_H_
