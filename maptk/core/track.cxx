/*ckwg +5
 * Copyright 2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include "track.h"

namespace
{
class compare_state_frame
{
public:
  bool operator()(const maptk::track::track_state& ts, unsigned int frame)
  {
    return ts.frame_id < frame;
  }
};
}


namespace maptk
{

/// Access the first frame number covered by this track
unsigned int
track
::first_frame() const
{
  if( this->history_.empty() )
  {
    return 0;
  }
  return this->history_.begin()->frame_id;
}


/// Access the last frame number covered by this track
unsigned int
track
::last_frame() const
{
  if( this->history_.empty() )
  {
    return 0;
  }
  return this->history_.rbegin()->frame_id;
}


/// Append a track state.
bool
track
::append(const track_state& state)
{
  if( !this->history_.empty() &&
      this->last_frame() >= state.frame_id )
  {
    return false;
  }
  this->history_.push_back(state);
  return true;
}


/// Find the track state iterator matching \a frame
track::history_const_itr
track
::find(unsigned int frame) const
{
  if( frame < this->first_frame() ||
      frame > this->last_frame() )
  {
    return this->end();
  }
  history_const_itr it = std::lower_bound(this->begin(), this->end(),
                                          frame, compare_state_frame());
  if( it != this->end() && it->frame_id == frame )
  {
    return it;
  }
  return this->end();
}


} // end namespace maptk
