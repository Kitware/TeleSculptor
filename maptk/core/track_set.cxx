/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief Implementation of \link maptk::track_set track_set \endlink
 *        member functions
 */

#include <maptk/core/track_set.h>
#include <boost/foreach.hpp>


namespace maptk
{

/// Return the number of tracks in the set
size_t
track_set
::size() const
{
  return this->tracks().size();
}


/// Return the set of all frame IDs covered by these tracks
std::set<frame_id_t>
track_set
::all_frame_ids() const
{
  std::set<frame_id_t> ids;
  const std::vector<track_sptr> all_tracks = this->tracks();
  BOOST_FOREACH(track_sptr t, all_tracks)
  {
    std::set<frame_id_t> t_ids = t->all_frame_ids();
    ids.insert(t_ids.begin(), t_ids.end());
  }
  return ids;
}


/// Return the set of all track IDs in this track set
std::set<track_id_t>
track_set
::all_track_ids() const
{
  std::set<track_id_t> ids;
  const std::vector<track_sptr> all_tracks = this->tracks();
  BOOST_FOREACH(track_sptr t, all_tracks)
  {
    ids.insert(t->id());
  }
  return ids;
}


/// Return the last (largest) frame number containing tracks
frame_id_t
track_set
::last_frame() const
{
  frame_id_t last_frame = 0;
  const std::vector<track_sptr> all_tracks = this->tracks();
  BOOST_FOREACH(track_sptr t, all_tracks)
  {
    if( t->last_frame() > last_frame )
    {
      last_frame = t->last_frame();
    }
  }
  return last_frame;
}


/// Return all tracks active on a frame.
track_set_sptr
track_set
::active_tracks(int offset)
{
  frame_id_t frame_number = offset_to_frame(offset);
  const std::vector<track_sptr> all_tracks = this->tracks();
  std::vector<track_sptr> active_tracks;
  BOOST_FOREACH(track_sptr t, all_tracks)
  {
    if( t->find(frame_number) != t->end() )
    {
      active_tracks.push_back(t);
    }
  }
  return track_set_sptr(new simple_track_set(active_tracks));
}


/// Return all tracks active on a frame.
track_set_sptr
track_set
::inactive_tracks(int offset)
{
  frame_id_t frame_number = offset_to_frame(offset);
  const std::vector<track_sptr> all_tracks = this->tracks();
  std::vector<track_sptr> inactive_tracks;
  BOOST_FOREACH(track_sptr t, all_tracks)
  {
    if( t->find(frame_number) == t->end() )
    {
      inactive_tracks.push_back(t);
    }
  }
  return track_set_sptr(new simple_track_set(inactive_tracks));
}


/// Return the set of features in tracks on the last frame
feature_set_sptr
track_set
::last_frame_features() const
{
  const frame_id_t last_frame = this->last_frame();
  std::vector<feature_sptr> last_features;
  const std::vector<track_sptr> all_tracks = this->tracks();
  std::vector<track_sptr> active_tracks;
  BOOST_FOREACH(track_sptr t, all_tracks)
  {
    if( t->last_frame() == last_frame )
    {
      last_features.push_back((t->end()-1)->feat);
    }
  }
  return feature_set_sptr(new simple_feature_set(last_features));
}


/// Return the set of descriptors in tracks on the last frame
descriptor_set_sptr
track_set
::last_frame_descriptors() const
{
  const frame_id_t last_frame = this->last_frame();
  std::vector<descriptor_sptr> last_descriptors;
  const std::vector<track_sptr> all_tracks = this->tracks();
  std::vector<track_sptr> active_tracks;
  BOOST_FOREACH(track_sptr t, all_tracks)
  {
    if( t->last_frame() == last_frame )
    {
      last_descriptors.push_back((t->end()-1)->desc);
    }
  }
  return descriptor_set_sptr(new simple_descriptor_set(last_descriptors));
}


/// Convert an offset number to an absolute frame number
frame_id_t
track_set
::offset_to_frame(int offset) const
{
  if( offset >= 0 )
  {
    return static_cast<frame_id_t>(offset);
  }

  frame_id_t frame_number = this->last_frame() + 1;
  if( static_cast<frame_id_t>(-offset) <= frame_number )
  {
    frame_number += offset;
  }
  return frame_number;
}


} // end namespace maptk
