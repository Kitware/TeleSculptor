/*ckwg +5
 * Copyright 2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_TRACK_H_
#define MAPTK_TRACK_H_

#include <vector>

#include <boost/shared_ptr.hpp>

#include "feature.h"
#include "descriptor.h"

namespace maptk
{

/// A representation of a feature track.
///
/// A track is a sequence of feature points and their descriptors
/// that represent a sequence of observations of the same world location.
/// A track is required to construct a landmark in 3D.
class track
{
public:
  typedef boost::shared_ptr<descriptor> descriptor_sptr;
  typedef boost::shared_ptr<feature> feature_sptr;

  /// A structure to hold the state of a track on a given frame
  struct track_state
  {
    /// Constructor
    track_state(unsigned int frame,
                feature_sptr feature,
                descriptor_sptr descriptor)
    : frame_id(frame),
      feat(feature),
      desc(descriptor) {}

    /// The frame identifier (i.e. frame number)
    unsigned int frame_id;
    /// The feature detected on frame \a frame_id
    feature_sptr feat;
    /// The descriptor extracted on frame \a frame_id
    descriptor_sptr desc;
  };

  typedef std::vector<track_state>::const_iterator history_const_itr;

  /// Default Constructor
  track();

  /// Access the first frame number covered by this track
  unsigned int first_frame() const;

  /// Access the last frame number covered by this track
  unsigned int last_frame() const;

  /// Append a track state.
  /// The added track state must have a frame_id greater than
  /// the last frame in the history.
  /// \returns true if successful, false not correctly ordered
  bool append(const track_state& state);

  /// Access a const iterator to the start of the history
  history_const_itr begin() const { return history_.begin(); }

  /// Access a const iterator to the end of the history
  history_const_itr end() const { return history_.end(); }

  /// Return the number of states in the track.
  size_t size() const { return history_.size(); }

protected:
  /// The ordered array of track states
  std::vector<track_state> history_;
};


} // end namespace maptk


#endif // MAPTK_TRACK_H_
