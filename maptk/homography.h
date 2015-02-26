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
 * \brief Core Homography class definition
 */

#ifndef MAPTK_HOMOGRAPHY_H_
#define MAPTK_HOMOGRAPHY_H_

#include <maptk/config.h>
#include <maptk/matrix.h>
#include <maptk/types.h>
#include <maptk/vector.h>

#include <iostream>
#include <map>
#include <vector>

#include <boost/shared_ptr.hpp>


namespace maptk
{


// Forward declarations of abstract homography class
class homography;
// typedef for a homography shared pointer
typedef boost::shared_ptr< homography > homography_sptr;


// ===========================================================================
// Homography Base-class
// ---------------------------------------------------------------------------

/// Abstract base homography transformation representation class
class homography
{
public:
  /// Destructor
  virtual ~homography() {}

  /// Create a clone of this homography object, returning as smart pointer
  /**
   * \return A new deep clone of this homography transformation.
   */
  virtual homography_sptr clone() const = 0;

  /// Get a double-typed copy of the underlying matrix transformation
  /**
   * \return A copy of the transformation matrix represented in the double
   *         type.
   */
  virtual Eigen::Matrix<double,3,3> matrix_d() const = 0;

  /// Get a float-typed copy of the underlying matrix transformation
  /**
   * \return A copy of the transformation matrix represented in the float
   *         type.
   */
  virtual Eigen::Matrix<float,3,3> matrix_f() const = 0;

  /// Get a new \p homography that has been normalized
  /**
   * Normalized \p homography is one in which the lower-right corner
   * (index (2,2]) is 1.
   *
   * If this index is 0, the nothing is modified.
   *
   * \return New homography transformation instance.
   */
  virtual homography_sptr normalize() const = 0;

  /// Get a new \p homography that has been inverted.
  /**
   * \return New homography transformation instance.
   */
  virtual homography_sptr inverse() const = 0;

};


// ===========================================================================
// Typed Homography
// ---------------------------------------------------------------------------

/// Representation of a matrix-based homography transformation
template <typename T>
class MAPTK_LIB_EXPORT homography_
  : public homography
{
public:
  typedef T value_type;
  typedef Eigen::Matrix<T,3,3> matrix_t;

  /// Construct an identity homography
  homography_<T>();

  /// Construct from a provided transformation matrix
  /**
   * \param mat The 3x3 transformation matrix to use.
   */
  homography_<T>( matrix_t const &mat );

  /// Copy constructor
  /**
   * \param other The other homography whose transformation should be copied.
   */
  template <typename U>
  homography_<T>( homography_<U> const &other );

  /// Copy constructor from sptr
  /**
   * \param other The other homography sptr
   */
  homography_<T>( homography_sptr other );

  // Abstract method definitions ---------------------------------------------

  /// Create a clone of ourself as a shared pointer
  /**
   * \return A new clone of this homography transformation.
   */
  virtual homography_sptr clone() const;

  /// Get a double-typed copy of the underlying matrix transformation
  /**
   * \return A copy of the transformation matrix represented in the double
   *         type.
   */
  virtual Eigen::Matrix<double,3,3> matrix_d() const;

  /// Get a float-typed copy of the underlying matrix transformation
  /**
   * \return A copy of the transformation matrix represented in the float
   *         type.
   */
  virtual Eigen::Matrix<float,3,3> matrix_f() const;


  /// Get a new \p homography that has been normalized
  /**
   * Normalized homography is one in which the lower-right corner (index (2,2])
   * is 1.
   *
   * If this index is 0, the nothing is modified.
   *
   * \return New homography transformation instance.
   */
  virtual homography_sptr normalize() const;

  /// Get a new \p homography that has been inverted.
  /**
   * \throws non_invertible_matrix When the homography matrix is non-invertible.
   * \return New homography transformation instance.
   */
  virtual homography_sptr inverse() const;

  // Member Functions --------------------------------------------------------

  /// Get the underlying matrix transformation
  /**
   * \return The reference to this homography's transformation matrix.
   */
  matrix_t& get_matrix();

  /// Get a const new copy of the underlying matrix transformation.
  matrix_t const& get_matrix() const;

  /// Custom multiplication operator that multiplies the underlying matrices
  /**
   * \param rhs Right-hand-side operand homography.
   * \return New homography object whose transform is the result of
   *         \p this * \p rhs.
   */
  virtual homography_<T> operator*( homography_<T> const &rhs );

protected:
  /// homography transformation matrix
  matrix_t h_;
};


// ===========================================================================
// Utility Functions
// ---------------------------------------------------------------------------

/// Homography mapping for 2D points.
template <typename T>
MAPTK_LIB_EXPORT
Eigen::Matrix<T,2,1>
homography_map( homography_<T> const &h, Eigen::Matrix<T,2,1> const &p );

/// Homography mapping for 2D points using sptr
template <typename T>
MAPTK_LIB_EXPORT
Eigen::Matrix<T,2,1>
homography_map( homography_sptr const &h, Eigen::Matrix<T,2,1> const &p );


/// Output stream operator for \p homography base-class
MAPTK_LIB_EXPORT std::ostream& operator<<( std::ostream &s, homography const &h );

/// homography_<T> output stream operator
template <typename T>
MAPTK_LIB_EXPORT std::ostream& operator<<( std::ostream &s, homography_<T> const &h );

/// Output stream operator for \p homography_sptr
MAPTK_LIB_EXPORT std::ostream& operator<<( std::ostream &s, homography_sptr h );


} // end namespace maptk

#endif // MAPTK_HOMOGRAPHY_H_
