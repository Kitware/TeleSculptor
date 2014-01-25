/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include "track.h"


/**
 * \file
 * \brief Implementation of \link maptk::track track \endlink
 */


namespace
{
class compare_state_frame
{
public:
  bool operator()(const maptk::track::track_state& ts, maptk::frame_id_t frame)
  {
    return ts.frame_id < frame;
  }
};
}


namespace maptk
{


/// Default Constructor
track
::track()
: id_(0)
{
}


/// Copy Constructor
track
::track(const track& other)
: history_(other.history_),
  id_(other.id_)
{
}


/// Construct a track from a single track state
track
::track(const track_state& ts)
: history_(1,ts),
  id_(0)
{
}


/// Access the first frame number covered by this track
frame_id_t
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
frame_id_t
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
::find(frame_id_t frame) const
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
