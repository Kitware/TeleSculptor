/*ckwg +5
 * Copyright 2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include "track.h"


namespace maptk
{

/// Access the first frame number covered by this track
template <typename T>
unsigned int
track_<T>
::first_frame() const
{
  if( this->history_.empty() )
  {
    return 0;
  }
  return this->history_.begin()->frame_id;
}


/// Access the last frame number covered by this track
template <typename T>
unsigned int
track_<T>
::last_frame() const
{
  if( this->history_.empty() )
  {
    return 0;
  }
  return this->history_.rbegin()->frame_id;
}


/// Append a track state.
template <typename T>
bool
track_<T>
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


#define INSTANTIATE_TRACK(T) \
template class track_<T>; \

INSTANTIATE_TRACK(double);
INSTANTIATE_TRACK(float);

#undef INSTANTIATE_TRACK
} // end namespace maptk
