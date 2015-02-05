/*ckwg +29
 * Copyright 2014-2015 by Kitware, Inc.
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
 * \brief core homography related classes implementation
 */

#include "homography.h"
#include "exceptions/math.h"
#include <Eigen/LU>


namespace maptk
{


// ===========================================================================
// homography_<T>
// ---------------------------------------------------------------------------


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

/// Get a float-typed copy of the underlying matrix transformation
template <typename T>
Eigen::Matrix<float,3,3>
homography_<T>
::matrix_f() const
{
  return this->h_.template cast<float>();
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
// f2f_homography_<T>
// ---------------------------------------------------------------------------

/// Construct an identity homography for the given frame
f2f_homography
::f2f_homography( frame_id_t const frame_id )
  : h_( homography_sptr( new homography_<double>() ) ),
    from_id_( frame_id ),
    to_id_( frame_id )
{
}

/// Construct a frame to frame homography given an existing transform
f2f_homography
::f2f_homography( homography_sptr const &h,
                  frame_id_t const from_id,
                  frame_id_t const to_id )
  : h_( h->clone() ),
    from_id_( from_id ),
    to_id_( to_id )
{
}

/// Copy constructor
f2f_homography
::f2f_homography( f2f_homography const &h )
  : h_( h.h_->clone() ),
    from_id_( h.from_id_ ),
    to_id_( h.to_id_ )
{
}

/// Destructor
f2f_homography
::~f2f_homography()
{
}

/// Get the homography transformation
homography_sptr
f2f_homography
::homography() const
{
  return this->h_;
}

/// Frame identifier that the homography maps from.
frame_id_t
f2f_homography
::from_id() const
{
  return this->from_id_;
}

/// Frame identifier that the homography maps to.
frame_id_t
f2f_homography
::to_id() const
{
  return this->to_id_;
}

/// Return a new inverse \p f2f_homography instance
f2f_homography
f2f_homography
::inverse() const
{
  homography_sptr h_inv( this->h_->clone() );
  h_inv->invert();
  return f2f_homography( h_inv, this->to_id_, this->from_id_ );
}

/// Custom f2f_homography multiplication operator for \p f2f_homography
f2f_homography
f2f_homography
::operator*( f2f_homography const &rhs )
{
  if( this->from_id() != rhs.to_id() )
  {
    throw invalid_matrix_operation( "Homography frame identifiers do not match up" );
  }

  Eigen::Matrix<double,3,3> new_h = this->h_->matrix_d() * rhs.h_->matrix_d();
  return f2f_homography( new_h, rhs.from_id(), this->to_id() );
}


//// ===========================================================================
//// f2w_homography_<T>
//// ---------------------------------------------------------------------------
//
//
///// Construct an identity homography for the given frame.
//template <typename T>
//f2w_homography_<T>
//::f2w_homography_( frame_id_t const frame_id )
//  : homography_<T>(),
//    frame_id_( frame_id )
//{
//}
//
///// Construct a frame to frame homography.
//template <typename T>
//f2w_homography_<T>
//::f2w_homography_( homography_<T> const &h,
//                   frame_id_t const frame_id )
//: homography_<T>( h ),
//  frame_id_( frame_id )
//{
//}
//
///// Copy Constructor.
//template <typename T>
//template <typename U>
//f2w_homography_<T>
//::f2w_homography_( f2w_homography_<U> const &h )
//: homography_<T>( h ),
//  frame_id_( h.frame_id() )
//{
//}
//
///// Create a clone of ourself as a shared pointer
//template <typename T>
//homography_sptr
//f2w_homography_<T>
//::clone() const
//{
//  return homography_sptr( new f2w_homography_<T>( *this ) );
//}
//
///// Get a double-typed copy of the underlying matrix transformation
//template <typename T>
//Eigen::Matrix<double,3,3>
//f2w_homography_<T>
//::matrix_d() const
//{
//  return homography_<T>::matrix_d();
//}
//
///// The frame identifier that this homography maps from.
//template <typename T>
//frame_id_t
//f2w_homography_<T>
//::frame_id() const
//{
//  return frame_id_;
//}


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
homography_map( homography_sptr h, Eigen::Matrix<T,2,1> const &p )
{
  // intentionally empty
}

template <>
MAPTK_LIB_EXPORT
Eigen::Matrix<double,2,1>
homography_map( homography_sptr h, Eigen::Matrix<double,2,1> const &p )
{
  return h_map<double>( h->matrix_d(), p );
}

template <>
MAPTK_LIB_EXPORT
Eigen::Matrix<float,2,1>
homography_map( homography_sptr h, Eigen::Matrix<float,2,1> const &p )
{
  return h_map<float>( h->matrix_f(), p );
}


// ===========================================================================
// Template class instantiation
// ---------------------------------------------------------------------------
/// \cond DoxygenSuppress
#define INSTANTIATE_HOMOGRAPHY(T) \
  template class MAPTK_LIB_EXPORT homography_<T>

#define INSTANTIATE_METHODS(T) \
  template MAPTK_LIB_EXPORT Eigen::Matrix<T,2,1> homography_map( homography_<T> const &h, \
                                                                 Eigen::Matrix<T,2,1> const &p )

INSTANTIATE_HOMOGRAPHY(float);
//INSTANTIATE_HOMOGRAPHY(double); // defined through use above (L:169)
INSTANTIATE_METHODS(float);
INSTANTIATE_METHODS(double);
#undef INSTANTIATE_HOMOGRAPHY
/// \endcond
// ===========================================================================


} // end namespace maptk
