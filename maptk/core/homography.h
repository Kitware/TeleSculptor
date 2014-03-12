/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief core homography related classes
 */

#ifndef MAPTK_HOMOGRAPHY_H_
#define MAPTK_HOMOGRAPHY_H_

#include "core_config.h"
#include "matrix.h"

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
class MAPTK_CORE_EXPORT f2f_homography : public homography
{
public:

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

protected:

  /// From frame identifier.
  frame_id_t from_id_;

  /// To frame identifier.
  frame_id_t to_id_;

};

/// A smart pointer to a frame homography.
typedef boost::shared_ptr< f2f_homography > f2f_homography_sptr;


/// A homography between a frame and some arbitrary coordinate space.
class MAPTK_CORE_EXPORT f2w_homography : public homography
{
public:

  /// Construct a frame to frame homography.
  explicit f2w_homography( const homography& h,
                           const frame_id_t frame_id );

  /// Copy Constructor.
  f2w_homography( const f2w_homography& h );

  /// Destructor.
  virtual ~f2w_homography();

  /// Return the inverse of this homography.
  virtual f2w_homography inverse() const;

  /// The frame identifier that this homography maps from.
  virtual frame_id_t frame_id() const;

protected:

  /// From frame identifier.
  frame_id_t frame_id_;

};

/// A smart pointer to a frame homography.
typedef boost::shared_ptr< f2w_homography > f2w_homography_sptr;


/// A point for use with multiplying with homography matrices.
class MAPTK_CORE_EXPORT homography_point
{
public:

  /// Constructor.
  homography_point( const double x, const double y );

  /// Destructor.
  virtual ~homography_point() {}

  /// Return x value.
  virtual double x() const;

  /// Return y value.
  virtual double y() const;

private:

  double x_, y_;
};

/// A smart pointer to a homography point.
typedef boost::shared_ptr< homography_point > homography_point_sptr;


/// A collection of homography-related data useful for each individual frame.
class MAPTK_CORE_EXPORT homography_collection
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


/// Custom homography point multiplication operator.
inline homography_point operator*( const homography& h, const homography_point& p );


} // end namespace maptk

#endif // MAPTK_HOMOGRAPHY_H_
