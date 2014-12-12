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
 * \brief core covariance interface
 */

#ifndef MAPTK_COVARIANCE_H_
#define MAPTK_COVARIANCE_H_

#include <maptk/config.h>

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

  /// Copy constructor
  covariance_<N,T>(const covariance_<N,T>& other)
  {
    memcpy( data_, other.data_, sizeof(data_) );
  }

  /// Copy Constructor from another type
  template <typename U>
  explicit covariance_<N,T>(const covariance_<N,U>& other)
  {
    const U* in = other.data();
    T* out = this->data_;
    for(unsigned i=0; i<N; ++i, ++in, ++out)
    {
      *out = static_cast<T>(*in);
    }
  }

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
  /**
   * Averages off diagonal elements to enforce symmetry
   * \param mat matrix to construct from.
   */
  explicit covariance_<N,T>(const Eigen::Matrix<T,N,N>& mat)
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

  /// Assignment operator
  covariance_<N,T>& operator=(const covariance_<N,T>& other)
  {
    memcpy( data_, other.data_, sizeof(data_) );
    return *this;
  }


  /// Extract a full matrix
  operator Eigen::Matrix<T,N,N>() const
  {
    Eigen::Matrix<T,N,N> mat;
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

  /// data of the sparse symmetric covarience matrix
  T data_[data_size];
};

/// \cond DoxygenSuppress
typedef covariance_<2,double> covariance_2d;
typedef covariance_<2,float> covariance_2f;
typedef covariance_<3,double> covariance_3d;
typedef covariance_<3,float> covariance_3f;
/// \endcond

/// output stream operator for a covariance
/**
 * \param s output stream
 * \param c covariance matrix to stream
 */
template <unsigned N, typename T>
//MAPTK_LIB_EXPORT
std::ostream& operator<<(std::ostream& s, const covariance_<N,T>& c);

/// input stream operator for a covariance
template <unsigned N, typename T>
//MAPTK_LIB_EXPORT
std::istream& operator>>(std::istream& s, covariance_<N,T>& c);


} // end namespace maptk


#endif // MAPTK_COVARIANCE_H_
