/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_TRACK_H_
#define MAPTK_TRACK_H_

#include "core_config.h"
#include "types.h"

#include <vector>

#include <boost/shared_ptr.hpp>

#include "descriptor.h"
#include "feature.h"

namespace maptk
{

/// A representation of a feature track.
/**
 * A track is a sequence of feature points and their descriptors
 * that represent a sequence of observations of the same world location.
 * A track is required to construct a landmark in 3D.
 */
class MAPTK_CORE_EXPORT track
{
public:
  typedef boost::shared_ptr<descriptor> descriptor_sptr;
  typedef boost::shared_ptr<feature> feature_sptr;

  /// A structure to hold the state of a track on a given frame
  struct track_state
  {
    /// Constructor
    track_state(frame_id_t frame,
                feature_sptr feature,
                descriptor_sptr descriptor)
    : frame_id(frame),
      feat(feature),
      desc(descriptor) {}

    /// The frame identifier (i.e. frame number)
    frame_id_t frame_id;
    /// The feature detected on frame \a frame_id
    feature_sptr feat;
    /// The descriptor extracted on frame \a frame_id
    descriptor_sptr desc;
  };

  typedef std::vector<track_state>::const_iterator history_const_itr;

  /// Default Constructor
  track();

  /// Copy Constructor
  track(const track& other);

  /// Construct a track from a single track state
  explicit track(const track_state& ts);

  /// Access the track identification number
  track_id_t id() const { return id_; }

  /// Set the track identification number
  void set_id(track_id_t id) { id_ = id; }

  /// Access the first frame number covered by this track
  frame_id_t first_frame() const;

  /// Access the last frame number covered by this track
  frame_id_t last_frame() const;

  /// Append a track state.
  /**
   * The added track state must have a frame_id greater than
   * the last frame in the history.
   * \returns true if successful, false not correctly ordered
   */
  bool append(const track_state& state);

  /// Access a const iterator to the start of the history
  history_const_itr begin() const { return history_.begin(); }

  /// Access a const iterator to the end of the history
  history_const_itr end() const { return history_.end(); }

  /// Find the track state iterator matching \a frame
  /**
   *  \param [in] frame the frame number to access
   *  \return an iterator at the frame if found, or end() if not
   */
  history_const_itr find(frame_id_t frame) const;

  /// Return the number of states in the track.
  size_t size() const { return history_.size(); }

protected:
  /// The ordered array of track states
  std::vector<track_state> history_;
  /// The unique track identification number
  track_id_t id_;
};


typedef boost::shared_ptr<track> track_sptr;

} // end namespace maptk


#endif // MAPTK_TRACK_H_
