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
 * \brief core essential matrix template implementations
 */

#include "essential_matrix.h"

#include <cmath>

#include <maptk/exceptions/math.h>
#include <maptk/logging_macros.h>

#include <Eigen/LU>


namespace maptk
{

/// Construct from a provided matrix
template <typename T>
essential_matrix_<T>
::essential_matrix_( Eigen::Matrix<T,3,3> const &mat )
  : h_( mat )
{
}

/// Conversion Copy constructor -- float specialization
template <>
template <>
essential_matrix_<float>
::essential_matrix_( essential_matrix_<float> const &other )
  : h_( other.get_matrix() )
{
}

/// Conversion Copy constructor -- double specialization
template <>
template <>
essential_matrix_<double>
::essential_matrix_( essential_matrix_<double> const &other )
  : h_( other.get_matrix() )
{
}

/// Construct from a generic essential_matrix
template <typename T>
essential_matrix_<T>
::essential_matrix_( essential_matrix const &base )
  : h_( base.matrix().template cast<T>() )
{
}

/// Construct from a generic essential_matrix -- double specialization
template <>
essential_matrix_<double>
::essential_matrix_( essential_matrix const &base )
  : h_( base.matrix() )
{
}

/// Create a clone of outself as a shared pointer
template <typename T>
essential_matrix_sptr
essential_matrix_<T>
::clone() const
{
  return essential_matrix_sptr( new essential_matrix_<T>( *this ) );
}

/// Get a double-typed copy of the underlying matrix
template <typename T>
Eigen::Matrix<double,3,3>
essential_matrix_<T>
::matrix() const
{
  return this->h_.template cast<double>();
}

/// Specialization for matrices with native double type
template <>
Eigen::Matrix<double,3,3>
essential_matrix_<double>
::matrix() const
{
  return this->h_;
}

/// Get the underlying matrix
template <typename T>
typename essential_matrix_<T>::matrix_t&
essential_matrix_<T>
::get_matrix()
{
  return this->h_;
}

/// Get a const reference to the underlying matrix.
template <typename T>
typename essential_matrix_<T>::matrix_t const&
essential_matrix_<T>
::get_matrix() const
{
  return this->h_;
}

// ===========================================================================
// Other Functions
// ---------------------------------------------------------------------------

/// essential_matrix_<T> output stream operator
template <typename T>
std::ostream&
operator<<( std::ostream &s, essential_matrix_<T> const &h )
{
  s << h.get_matrix();
  return s;
}

/// Output stream operator for \p essential_matrix instances
std::ostream&
operator<<( std::ostream &s, essential_matrix const &h )
{
  s << h.matrix();
  return s;
}

// ===========================================================================
// Template class instantiation
// ---------------------------------------------------------------------------
/// \cond DoxygenSuppress
#define INSTANTIATE_ESSENTIAL_MATRIX(T) \
  template class essential_matrix_<T>; \
  template MAPTK_LIB_EXPORT std::ostream& operator<<( std::ostream &, \
                                                      essential_matrix_<T> const & )

INSTANTIATE_ESSENTIAL_MATRIX(float);
INSTANTIATE_ESSENTIAL_MATRIX(double);
#undef INSTANTIATE_ESSENTIAL_MATRIX
/// \endcond


} // end maptk namespace
