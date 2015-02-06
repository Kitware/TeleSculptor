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
 * \brief core homography template implementations
 */

#include "homography.h"

#include <maptk/exceptions/math.h>

#include <Eigen/LU>


namespace maptk
{


/// Construct an identity homography
template <typename T>
homography_<T>
::homography_()
  : h_( matrix_t::Identity() )
{
}

/// Construct from a provided transformation matrix
template <typename T>
homography_<T>
::homography_( matrix_t const &mat )
  : h_( mat )
{
}

/// Copy constructor
template <typename T>
template <typename U>
homography_<T>
::homography_( homography_<U> const &other )
  : h_( other.get_matrix().template cast<T>() )
{
}

/// Create a clone of outself as a shared pointer
template <typename T>
homography_sptr
homography_<T>
::clone() const
{
  return homography_sptr( new homography_<T>( *this ) );
}

/// Get a double-typed copy of the underlying matrix transformation
template <typename T>
Eigen::Matrix<double,3,3>
homography_<T>
::matrix_d() const
{
  return this->h_.template cast<double>();
}

/// Specialization for homographies with native double type
template <>
Eigen::Matrix<double,3,3>
homography_<double>
::matrix_d() const
{
  return this->h_;
}

/// Get a float-typed copy of the underlying matrix transformation
template <typename T>
Eigen::Matrix<float,3,3>
homography_<T>
::matrix_f() const
{
  return this->h_.template cast<float>();
}

/// Specialization for homographies with native float type
template <>
Eigen::Matrix<float,3,3>
homography_<float>
::matrix_f() const
{
  return this->h_;
}

/// Normalize homography transformation in-place
template <typename T>
homography_<T>&
homography_<T>
::normalize()
{
  if( this->h_(2,2) != 0 )
  {
    this->h_ /= this->h_(2,2);
  }
  return *this;
}

/// Inverse the homography transformation returning a new transformation
template <typename T>
homography_<T>&
homography_<T>
::invert()
{
  matrix_t inv;
  bool isvalid;
  this->h_.computeInverseWithCheck( inv, isvalid );
  if( !isvalid )
  {
    throw non_invertible_matrix();
  }
  else
  {
    this->h_ = inv;
  }
  return *this;
}

template <typename T>
homography_sptr
homography_<T>
::Zero()
{
  return homography_sptr( new homography_<T>( matrix_t::Zero() ) );
}

/// Get the underlying matrix transformation
template <typename T>
typename homography_<T>::matrix_t&
homography_<T>
::get_matrix()
{
  return this->h_;
}

/// Get a const new copy of the underlying matrix transformation.
template <typename T>
typename homography_<T>::matrix_t const&
homography_<T>
::get_matrix() const
{
  return this->h_;
}

  /// Return a new homography with the inverse transformation
template <typename T>
homography_<T>
homography_<T>
::inverse() const
{
  matrix_t inv;
  bool isvalid;
  this->h_.computeInverseWithCheck( inv, isvalid );
  if( !isvalid )
  {
    throw non_invertible_matrix();
  }
  else
  {
    return homography_<T>( inv );
  }
}

/// Custom f2f_homography multiplication operator.
template <typename T>
homography_<T>
homography_<T>
::operator*( homography_<T> const &rhs )
{
  return homography_<T>( this->h_ * rhs.h_ );
}


// ===========================================================================
// Other Functions
// ---------------------------------------------------------------------------

namespace //anonymous
{

// base homog-point transformation logic
template <typename T>
Eigen::Matrix<T,2,1>
h_map( Eigen::Matrix<T,3,3> const &h, Eigen::Matrix<T,2,1> const &p )
{
  Eigen::Matrix<T,3,1> out_pt = h * Eigen::Matrix<T,3,1>(p[0], p[1], 1.0);
  if( out_pt[2] == 0 )
  {
    throw point_maps_to_infinity();
  }
  return Eigen::Matrix<T,2,1>( out_pt[0] / out_pt[2], out_pt[1] / out_pt[2] );
}

} // end anonymous namespace


template <typename T>
Eigen::Matrix<T,2,1>
homography_map( homography_<T> const &h, Eigen::Matrix<T,2,1> const &p )
{
  return h_map<T>( h.get_matrix(), p );
}


template <typename T>
Eigen::Matrix<T,2,1>
homography_map( homography_sptr const &h, Eigen::Matrix<T,2,1> const &p )
{
  // intentionally empty for C++ reasons.
}

template <>
MAPTK_LIB_EXPORT
Eigen::Matrix<double,2,1>
homography_map( homography_sptr const &h, Eigen::Matrix<double,2,1> const &p )
{
  return h_map<double>( h->matrix_d(), p );
}

template <>
MAPTK_LIB_EXPORT
Eigen::Matrix<float,2,1>
homography_map( homography_sptr const &h, Eigen::Matrix<float,2,1> const &p )
{
  return h_map<float>( h->matrix_f(), p );
}

/// homography_<T> output stream operator
template <typename T>
std::ostream&
operator<<( std::ostream &s, homography_<T> const &h )
{
  s << h.get_matrix();
  return s;
}


} // end maptk namespace
