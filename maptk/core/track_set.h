/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_TRACK_SET_H_
#define MAPTK_TRACK_SET_H_

#include "core_config.h"

#include <vector>

#include <boost/shared_ptr.hpp>

#include "descriptor_set.h"
#include "feature_set.h"
#include "track.h"

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

  /// Return the last (largest) frame number containing tracks
  virtual frame_id_t last_frame() const;

  /// Return all tracks active on a frame.
  /**
   * Active tracks are any tracks which contain a state on the target frame.
   * \param [in] offset the frame offset for selecting the active frame.
   *                    Postive number are absolute frame numbers while negative
   *                    numbers are relative to the last frame.  For example,
   *                    offset of -1 refers to the last frame and is the default.
   * \returns a track set which is the subset of tracks that are active.
   */
  virtual track_set_sptr active_tracks(int offset = -1);

  /// Return all tracks inactive on a frame.
  /**
   * Inactive tracks are any tracks which do not contain a state on the target frame.
   * \param [in] offset the frame offset for selecting the active frame.
   *                    Postive number are absolute frame numbers while negative
   *                    numbers are relative to the last frame.  For example,
   *                    offset of -1 refers to the last frame and is the default.
   * \returns a track set which is the subset of tracks that are inactive.
   */
  virtual track_set_sptr inactive_tracks(int offset = -1);

  /// Return the set of features in tracks on the last frame
  virtual feature_set_sptr last_frame_features() const;

  /// Return the set of descriptors in tracks on the last frame
  virtual descriptor_set_sptr last_frame_descriptors() const;

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
