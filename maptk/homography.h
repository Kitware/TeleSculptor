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
 * \brief core homography related classes
 */

#ifndef MAPTK_HOMOGRAPHY_H_
#define MAPTK_HOMOGRAPHY_H_

#include <maptk/config.h>
#include <maptk/types.h>

#include "matrix.h"
#include "vector.h"

#include <iostream>
#include <map>
#include <vector>

#include <boost/shared_ptr.hpp>

namespace maptk
{


// ===========================================================================
// Homography Type
// ---------------------------------------------------------------------------

// Forward declarations of abstract homography classe
class homography;
// typedef for a homography shared pointer
typedef boost::shared_ptr< homography > homography_sptr;


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

  /// Normalize \p homography transformation in-place
  /**
   * Normalized \p homography is one in which the lower-right corner
   * (index (2,2]) is 1.
   *
   * If this index is 0, the nothing is modified.
   *
   * \return The reference to ourself.
   */
  virtual homography& normalize() = 0;

  /// Invert \p homography transformation in-place
  virtual homography& invert() = 0;

};


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


  /// Normalize homography transformation in-place
  /**
   * Normalized homography is one in which the lower-right corner (index (2,2])
   * is 1.
   *
   * If this index is 0, the nothing is modified.
   *
   * \return The reference to ourself.
   */
  virtual homography_<T>& normalize();

  /// Inverse the homography transformation returning a new transformation
  virtual homography_<T>& invert();

  // Member Functions --------------------------------------------------------

  /// Return a new homography trasnformation composed of zeros
  static homography_sptr Zero();

  /// Get the underlying matrix transformation
  /**
   * \return The reference to this homography's transformation matrix.
   */
  matrix_t& get_matrix();

  /// Get a const new copy of the underlying matrix transformation.
  matrix_t const& get_matrix() const;

  /// Return a new homography with the inverse transformation
  virtual homography_<T> inverse() const;

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
// Frame-to-frame homography container
// ---------------------------------------------------------------------------


class MAPTK_LIB_EXPORT f2f_homography
{
public:
  /// Construct an identity homography for the given frame
  /**
   * \tparam T Data type for the underlying homography transformation
   */
  explicit f2f_homography( frame_id_t const frame_id );

  /// Construct a frame to frame homography using a matrix
  /**
   * \tparam T Data type for the underlying homography transformation
   */
  template <typename T>
  explicit f2f_homography( Eigen::Matrix<T,3,3> const &h,
                           frame_id_t const from_id,
                           frame_id_t const to_id )
    : h_( homography_sptr( new homography_<T>( h ) ) ),
      from_id_( from_id ),
      to_id_( to_id )
  {}


  /// Construct a frame to frame homography given an existing transform
  /**
   * The given homography sptr is cloned into this object so we retain a unique
   * copy.
   */
  explicit f2f_homography( homography_sptr const &h,
                           frame_id_t const from_id,
                           frame_id_t const to_id );

  /// Copy constructor
  f2f_homography( f2f_homography const &h );

  /// Destructor
  virtual ~f2f_homography();

  /// Get the sptr of the contained homography transformation
  virtual homography_sptr homography() const;

  /// Frame identifier that the homography maps from.
  virtual frame_id_t from_id() const;

  /// Frame identifier that the homography maps to.
  virtual frame_id_t to_id() const;

  /// Return a new inverse \p f2f_homography instance
  /**
   * \return New \p f2f_homography instance whose transformation is inverted as
   *         well as has flipped from and to ID references.
   */
  virtual f2f_homography inverse() const;

  /// Custom f2f_homography multiplication operator for \p f2f_homography
  /**
   * \throws invalid_matrix_operation
   *    When \p this.from_id() != \p rhs.to_id() as transformed from and to IDs
   *    are undefined otherwise.
   *
   * \param rhs Right-hand-side operand homography.
   * \return New homography object whose transform is the result of
   *         \p this * \p rhs.
   */
  virtual f2f_homography operator*( f2f_homography const &rhs );

protected:
  /// Homography transformation sptr.
  homography_sptr h_;

  /// From frame identifier.
  frame_id_t from_id_;

  /// To frame identifier.
  frame_id_t to_id_;
};

/// Shared pointer for \p f2f_homography
typedef boost::shared_ptr< f2f_homography > f2f_homography_sptr;


// ===========================================================================
// Frame to World Homography container
// ---------------------------------------------------------------------------


class MAPTK_LIB_EXPORT f2w_homography
{
public:
  /// Construct an identity homography for the given frame
  explicit f2w_homography( frame_id_t const frame_id );

  /// Construct given an existing homography
  /**
   * The given homography sptr is cloned into this object so we retain a unique
   * copy.
   */
  f2w_homography( homography_sptr const &h, frame_id_t const frame_id );

  /// Copy Constructor
  f2w_homography( f2w_homography const &h );

  /// Get the homography transformation
  virtual homography_sptr homography() const;

  /// Get the frame identifier
  virtual frame_id_t frame_id() const;

protected:
  /// Homography transformation
  homography_sptr h_;

  /// Frame identifier
  frame_id_t frame_id_;
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


/// Output stream operator for \p homography_sptr
MAPTK_LIB_EXPORT std::ostream& operator<<( std::ostream &s, homography const &h );

/// homography_<T> output stream operator
template <typename T>
MAPTK_LIB_EXPORT std::ostream& operator<<( std::ostream &s, homography_<T> const &h );

/// \p f2f_homography output stream operator
MAPTK_LIB_EXPORT std::ostream& operator<<( std::ostream &s, f2f_homography const &h );


} // end namespace maptk

#endif // MAPTK_HOMOGRAPHY_H_
