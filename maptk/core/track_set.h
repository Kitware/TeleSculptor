/*ckwg +5
 * Copyright 2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_TRACK_SET_H_
#define MAPTK_TRACK_SET_H_


#include "track.h"
#include <vector>
#include <boost/shared_ptr.hpp>

namespace maptk
{

/// A collection of 2D feature point tracks
class track_set
{
public:
  /// Destructor
  virtual ~track_set() {}

  /// Return the number of tracks in the set
  virtual size_t size() const = 0;

  /// Return a vector of track shared pointers
  virtual std::vector<track_sptr> tracks() const = 0;
};

typedef boost::shared_ptr<track_set> track_set_sptr;


/// A concrete track set that simply wraps a vector of tracks.
class simple_track_set
: public track_set
{
public:
  /// Default Constructor
  simple_track_set() {}

  /// Constructor from a vector of tracks
  explicit simple_track_set(const std::vector<track_sptr>& tracks)
  : data_(tracks) {}

  /// Return the number of tracks in the set
  virtual size_t size() const { return data_.size(); }

  /// Return a vector of track shared pointers
  virtual std::vector<track_sptr> tracks() const { return data_; }

protected:

  /// The vector of tracks
  std::vector<track_sptr> data_;
};


} // end namespace maptk


#endif // MAPTK_TRACK_SET_H_
