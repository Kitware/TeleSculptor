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
 * \brief Frame to Frame Homography definition
 */

#ifndef MAPTK_HOMOGRAPHY_F2F_H
#define MAPTK_HOMOGRAPHY_F2F_H

#include <maptk/homography.h>


namespace maptk
{


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


/// \p f2f_homography output stream operator
MAPTK_LIB_EXPORT std::ostream& operator<<( std::ostream &s, f2f_homography const &h );


} // end maptk namespace

#endif // MAPTK_HOMOGRAPHY_F2F_H
