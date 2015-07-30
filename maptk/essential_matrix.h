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
 * \brief Core essential matrix class definition
 */

#ifndef MAPTK_ESSENTIAL_MATRIX_H_
#define MAPTK_ESSENTIAL_MATRIX_H_

#include <maptk/config.h>
#include <maptk/matrix.h>
#include <maptk/types.h>
#include <maptk/vector.h>
#include <maptk/rotation.h>

#include <iostream>
#include <map>
#include <vector>

#include <boost/shared_ptr.hpp>


namespace maptk
{


// Forward declarations of abstract essential matrix class
class essential_matrix;
// typedef for a essential_matrix shared pointer
typedef boost::shared_ptr< essential_matrix > essential_matrix_sptr;


// ===========================================================================
// Essential Matrix Base-class
// ---------------------------------------------------------------------------

/// Abstract base essential matrix representation class
class essential_matrix
{
public:
  /// Destructor
  virtual ~essential_matrix() MAPTK_DEFAULT_DTOR;

  /// Create a clone of this essential_matrix object, returning as smart pointer
  /**
   * \return A new deep clone of this essential_matrix transformation.
   */
  virtual essential_matrix_sptr clone() const = 0;

  /// Get a double-typed copy of the underlying matrix
  /**
   * \return A copy of the matrix represented in the double type.
   */
  virtual matrix_3x3d matrix() const = 0;

  /// Return the one of two possible 3D rotations that can parameterize E
  virtual rotation_d rotation() const = 0;

  /// Return the second possible rotation that can parameterize E
  /**
   *  The twisted rotation is related to the primary rotation by a 180 degree
   *  rotation about the translation axis
   */
  virtual rotation_d twisted_rotation() const;

  /// Return a unit translation vector (up to a sign) that parameterizes E
  virtual vector_3d translation() const = 0;
};


// ===========================================================================
// Typed Essential Matrix
// ---------------------------------------------------------------------------

/// Representation of a templated Eigen-based essential matrix
template <typename T>
class MAPTK_LIB_EXPORT essential_matrix_
  : public essential_matrix
{
public:
  typedef T value_type;
  typedef Eigen::Matrix<T,3,3> matrix_t;
  typedef Eigen::Matrix<T,3,1> vector_t;

  /// Construct from a provided matrix by projection.
  /** Decompose and find closest essential matrix to the input \p mat.
   * \param mat The 3x3 transformation matrix to use.
   */
  explicit
  essential_matrix_<T>( matrix_t const &mat );

  /// Construct from a rotation and translation
  essential_matrix_<T>( rotation_<T> const &rot,
                        vector_t const &trans );

  /// Conversion Copy constructor
  /**
   * \param other The other essential_matrix to be copied.
   */
  template <typename U>
  explicit
  essential_matrix_<T>( essential_matrix_<U> const &other )
    : rot_( static_cast<rotation_<T> >(other.rot_) ),
      trans_( other.trans_.template cast<T>() )
  {
  }

  /// Construct from a generic essential_matrix
  explicit
  essential_matrix_<T>( essential_matrix const &base );

  // Abstract method definitions ---------------------------------------------

  /// Create a clone of ourself as a shared pointer
  /**
   * \return A new clone of this essential_matrix.
   */
  virtual essential_matrix_sptr clone() const;

  /// Get a double-typed copy of the underlying matrix
  /**
   * \return A copy of the matrix represented in the double type.
   */
  virtual Eigen::Matrix<double,3,3> matrix() const;

  /// Return the one of two possible 3D rotations that can parameterize E
  virtual rotation_d rotation() const;

  /// Return the second possible rotation that can parameterize E
  /**
   *  The twisted rotation is related to the primary rotation by a 180 degree
   *  rotation about the translation axis
   */
  virtual rotation_d twisted_rotation() const;

  /// Return a unit translation vector (up to a sign) that parameterizes E
  virtual vector_3d translation() const;

  // Member Functions --------------------------------------------------------

  /// Compute the matrix representation from rotatation and translation
  matrix_t compute_matrix() const;

  /// Compute the twisted pair rotation from the rotation and translation
  /**
   *  The twisted rotation is related to the primary rotation by a 180 degree
   *  rotation about the translation axis
   */
  rotation_<T> compute_twisted_rotation() const;

  /// Get a const reference to the underlying rotation
  rotation_<T> const& get_rotation() const;

  /// Get a const reference to the underlying translation
  vector_t const& get_translation() const;

protected:
  /// the rotation used to parameterize the essential matrix
  rotation_<T> rot_;
  /// the translation used to parameterize the essential  matrix
  vector_t trans_;
};


/// Double-precision camera type
typedef essential_matrix_<double> essential_matrix_d;
/// Single-precision camera type
typedef essential_matrix_<float> essential_matrix_f;


// ===========================================================================
// Utility Functions
// ---------------------------------------------------------------------------

/// Output stream operator for \p essential_matrix base-class
MAPTK_LIB_EXPORT std::ostream& operator<<( std::ostream &s,
                                           essential_matrix const &e );

/// essential_matrix_<T> output stream operator
template <typename T>
MAPTK_LIB_EXPORT std::ostream& operator<<( std::ostream &s,
                                           essential_matrix_<T> const &e );


} // end namespace maptk

#endif // MAPTK_ESSENTIAL_MATRIX_H_
