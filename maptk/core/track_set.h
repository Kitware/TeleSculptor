/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_TRACK_SET_H_
#define MAPTK_TRACK_SET_H_

#include "core_config.h"

#include <vector>
#include <set>

#include <boost/shared_ptr.hpp>

#include "descriptor_set.h"
#include "feature_set.h"
#include "track.h"


/**
 * \file
 * \brief Header file for an abstract \link maptk::track_set track_set
 *        \endlink and a concrete \link maptk::simple_track_set
 *        simple_track_set \endlink
 */


namespace maptk
{

class track_set;
typedef boost::shared_ptr<track_set> track_set_sptr;

/// A collection of 2D feature point tracks
class MAPTK_CORE_EXPORT track_set
{
public:
  /// Destructor
  virtual ~track_set() {}

  /// Return the number of tracks in the set
  virtual size_t size() const;

  /// Return a vector of track shared pointers
  virtual std::vector<track_sptr> tracks() const = 0;

  /// Return the set of all frame IDs covered by these tracks
  virtual std::set<frame_id_t> all_frame_ids() const;

  /// Return the set of all track IDs in this track set
  virtual std::set<track_id_t> all_track_ids() const;

  /// Return the last (largest) frame number containing tracks
  virtual frame_id_t last_frame() const;

  /// Return all tracks active on a frame.
  /**
   * Active tracks are any tracks which contain a state on the target frame.
   *
   * \param [in] offset the frame offset for selecting the active frame.
   *                    Postive number are absolute frame numbers while negative
   *                    numbers are relative to the last frame.  For example,
   *                    offset of -1 refers to the last frame and is the default.
   *
   * \returns a track set which is the subset of tracks that are active.
   */
  virtual track_set_sptr active_tracks(int offset = -1);

  /// Return all tracks inactive on a frame.
  /**
   * Inactive tracks are any tracks which do not contain a state on the target frame.
   *
   * \param [in] offset the frame offset for selecting the active frame.
   *                    Postive number are absolute frame numbers while negative
   *                    numbers are relative to the last frame.  For example,
   *                    offset of -1 refers to the last frame and is the default.
   *
   * \returns a track set which is the subset of tracks that are inactive.
   */
  virtual track_set_sptr inactive_tracks(int offset = -1);

  /// Return all tracks newly initialized on the given frame.
  /**
   * New tracks include any tracks with a first track state on the target frame.
   *
   * \param [in] offset the frame offset for selecting the active frame.
   *                    Postive number are absolute frame numbers while negative
   *                    numbers are relative to the last frame.  For example,
   *                    offset of -1 refers to the last frame and is the default.
   *
   * \returns a track set containing all new tracks for the given frame.
   */
  virtual track_set_sptr new_tracks(int offset = -1);

  /// Return all tracks terminated on the given frame.
  /**
   * Terminated tracks include any tracks with a last track state on the frame.
   *
   * \param [in] offset the frame offset for selecting the active frame.
   *                    Postive number are absolute frame numbers while negative
   *                    numbers are relative to the last frame.  For example,
   *                    offset of -1 refers to the last frame and is the default.
   *
   * \returns a track set containing all terminated tracks for the given frame.
   */
  virtual track_set_sptr terminated_tracks(int offset = -1);

  /// Return the percentage of tracks successfully tracked between the two frames.
  /**
   * The percentage of tracks successfully tracked between frames is defined as the
   * number of tracks which have a track state on both frames, divided by the total
   * number of unique tracks which appear on both frames.
   *
   * \param [in] offset1 the frame offset for the first frame in the operation.
   *                     Postive number are absolute frame numbers while negative
   *                     numbers are relative to the last frame.  For example,
   *                     offset of -1 refers to the last frame and is the default.
   * \param [in] offset2 the frame offset for the second frame in the operation.
   *                     Postive number are absolute frame numbers while negative
   *                     numbers are relative to the last frame.  For example,
   *                     offset of -1 refers to the last frame and is the default.
   *
   * \returns a floating point percent value (between 0.0 and 1.0).
   */
  virtual double percentage_tracked(int offset1 = -2, int offset2 = -1);

  /// Return the set of features in tracks on the last frame
  virtual feature_set_sptr last_frame_features() const;

  /// Return the set of descriptors in tracks on the last frame
  virtual descriptor_set_sptr last_frame_descriptors() const;

  /// Return the set of features in all tracks for the given frame.
  /**
   * \param [in] offset the frame offset for selecting the target frame.
   *                    Postive number are absolute frame numbers while negative
   *                    numbers are relative to the last frame.  For example,
   *                    offset of -1 refers to the last frame and is the default.
   *
   * \returns a feature_set_sptr for all features on the give frame.
   */
  virtual feature_set_sptr frame_features(int offset = -1) const;

  /// Return the set of descriptors in all tracks for the given frame.
  /**
   * \param [in] offset the frame offset for selecting the target frame.
   *                    Postive number are absolute frame numbers while negative
   *                    numbers are relative to the last frame.  For example,
   *                    offset of -1 refers to the last frame and is the default.
   *
   * \returns a descriptor_set_sptr for all features on the give frame.
   */
  virtual descriptor_set_sptr frame_descriptors(int offset = -1) const;

protected:
  /// Convert an offset number to an absolute frame number
  frame_id_t offset_to_frame(int offset) const;
};



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
