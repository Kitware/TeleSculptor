/*ckwg +29
 * Copyright 2014 by Kitware, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither name of Kitware, Inc. nor the names of any contributors may be used
 *    to endorse or promote products derived from this software without specific
 *    prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHORS OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * \file
 * \brief Implementation of \link maptk::track track \endlink
 */

#include "track.h"
#include <boost/foreach.hpp>


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


/// Append an entire other track.
bool
track
::append(const track& to_append)
{
  if( !this->history_.empty() && !to_append.empty() &&
      this->last_frame() >= to_append.first_frame() )
  {
    return false;
  }
  this->history_.insert(this->history_.end(), to_append.begin(), to_append.end());
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


/// Return the set of all frame IDs covered by this track
std::set<frame_id_t>
track
::all_frame_ids() const
{
  std::set<frame_id_t> ids;
  BOOST_FOREACH(const track_state& ts, this->history_)
  {
    ids.insert(ts.frame_id);
  }
  return ids;
}


} // end namespace maptk
