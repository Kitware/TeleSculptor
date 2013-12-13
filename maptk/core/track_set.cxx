/*ckwg +5
 * Copyright 2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
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


/// Return the last (largest) frame number containing tracks
unsigned int
track_set
::last_frame() const
{
  unsigned int last_frame = 0;
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
  unsigned int frame_number;
  if( offset >= 0 )
  {
    frame_number = static_cast<unsigned int>(offset);
  }
  else
  {
    frame_number = this->last_frame() + 1;
    if( -offset <= frame_number )
    {
      frame_number += offset;
    }
  }

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


/// Return the set of features in tracks on the last frame
feature_set_sptr
track_set
::last_frame_features() const
{
  const unsigned int last_frame = this->last_frame();
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
  const unsigned int last_frame = this->last_frame();
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

} // end namespace maptk
