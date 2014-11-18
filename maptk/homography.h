/*ckwg +29
 * Copyright 2014 by Kitware, Inc.
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

#include "matrix.h"
#include "vector.h"

#include <iostream>
#include <map>
#include <vector>

#include <boost/shared_ptr.hpp>

namespace maptk
{


/// A raw homography transformation matrix.
typedef matrix_3x3d homography;

/// A smart pointer to a raw homography transformation matrix.
typedef boost::shared_ptr< homography > homography_sptr;


/// A homography between two arbitrary frames.
class MAPTK_LIB_EXPORT f2f_homography : public homography
{
public:

  /// Construct an identity homography for the given frame.
  explicit f2f_homography( const frame_id_t frame_id );

  /// Construct a frame to frame homography.
  explicit f2f_homography( const homography& h,
                           const frame_id_t from_id,
                           const frame_id_t to_id );

  /// Copy Constructor.
  f2f_homography( const f2f_homography& h );

  /// Destructor.
  virtual ~f2f_homography();

  /// Return the inverse of this homography.
  virtual f2f_homography inverse() const;

  /// The frame identifier that this homography maps from.
  virtual frame_id_t from_id() const;

  /// The frame identifier that this homography maps to.
  virtual frame_id_t to_id() const;

  /// Normalize the internal homography matrix.
  virtual f2f_homography& normalize();

  /// Custom f2f_homography multiplication operator.
  virtual f2f_homography operator*( const f2f_homography& other );

protected:

  /// From frame identifier.
  frame_id_t from_id_;

  /// To frame identifier.
  frame_id_t to_id_;

};

/// A smart pointer to a frame to frame homography.
typedef boost::shared_ptr< f2f_homography > f2f_homography_sptr;


/// A homography between a frame and some arbitrary coordinate space.
class MAPTK_LIB_EXPORT f2w_homography : public homography
{
public:

  /// Construct an identity homography for the given frame.
  explicit f2w_homography( const frame_id_t frame_id );

  /// Construct a frame to frame homography.
  explicit f2w_homography( const homography& h,
                           const frame_id_t frame_id );

  /// Copy Constructor.
  f2w_homography( const f2w_homography& h );

  /// Destructor.
  virtual ~f2w_homography();

  /// The frame identifier that this homography maps from.
  virtual frame_id_t frame_id() const;

protected:

  /// From frame identifier.
  frame_id_t frame_id_;

};

/// A smart pointer to a frame to world homography.
typedef boost::shared_ptr< f2w_homography > f2w_homography_sptr;


/// A collection of homography-related data useful for each individual frame.
class MAPTK_LIB_EXPORT homography_collection
{
public:

  /// Construct a homography collection from different types of homographies.
  homography_collection( f2f_homography_sptr cur_to_last = f2f_homography_sptr(),
                         f2f_homography_sptr cur_to_ref = f2f_homography_sptr(),
                         f2w_homography_sptr ref_to_wld = f2w_homography_sptr(),
                         f2w_homography_sptr cur_to_wld = f2w_homography_sptr() );

  /// Destructor.
  virtual ~homography_collection();

  /// Return a homography to the last frame.
  f2f_homography_sptr current_to_last() const;

  /// Return a homography to some reference frame.
  f2f_homography_sptr current_to_reference() const;

  /// Return an arbitrary reference to world homography.
  f2w_homography_sptr reference_to_world() const;

  /// Return a homography to some reference frame.
  f2w_homography_sptr current_to_world() const;

  /// Is the current to last homography valid?
  bool has_current_to_last() const;

  /// Is the current to reference homography valid?
  bool has_current_to_reference() const;

  /// Is the reference to world homography valid?
  bool has_reference_to_world() const;

  /// Is the current to world homography valid?
  bool has_current_to_world() const;

protected:

  /// The actual current to last homography.
  f2f_homography_sptr current_to_last_;

  /// The actual current to reference homography.
  f2f_homography_sptr current_to_reference_;

  /// The actual reference to world homography.
  f2w_homography_sptr reference_to_world_;

  /// The actual current to world homography.
  f2w_homography_sptr current_to_world_;

};

/// A pointer to a homography collection class.
typedef boost::shared_ptr< homography_collection> homography_collection_sptr;

/// A set of homography collections for several individual frames.
typedef std::map< frame_id_t, homography_collection > homography_collection_set;

/// A pointer to a homography_collection_set.
typedef boost::shared_ptr< homography_collection_set > homography_collection_set_sptr;


/// Custom homography multiplication operator for 2D points.
vector_2d operator*( const homography& h, const vector_2d& p );


} // end namespace maptk

#endif // MAPTK_HOMOGRAPHY_H_
