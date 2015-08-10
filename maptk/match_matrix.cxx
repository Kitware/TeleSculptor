/*ckwg +29
 * Copyright 2015 by Kitware, Inc.
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
 * \brief Implementation of match matrix computation
 */

#include "match_matrix.h"
#include <boost/foreach.hpp>
#include <map>


namespace maptk
{


/// Compute the match matrix from a track set
Eigen::MatrixXi
match_matrix(const track_set_sptr tracks,
                   std::vector<frame_id_t>& frames)
{
  // if no frames ids specified then get all frame ids in the track set
  if( frames.empty() )
  {
    std::set<frame_id_t> frame_ids = tracks->all_frame_ids();
    frames = std::vector<frame_id_t>(frame_ids.begin(), frame_ids.end());
  }
  const size_t num_frames = frames.size();

  // build a frame map for revsere lookup of matrix indices
  std::map<frame_id_t, unsigned int> frame_map;
  for( unsigned int i=0; i<num_frames; ++i )
  {
    frame_map[frames[i]] = i;
  }

  Eigen::MatrixXi mm = Eigen::MatrixXi::Zero(num_frames, num_frames);

  const std::vector<track_sptr> trks = tracks->tracks();
  BOOST_FOREACH(const track_sptr& t, trks)
  {
    // get all the frames covered by this track
    std::set<frame_id_t> t_frames = t->all_frame_ids();
    // map the frames to a vector of all valid matrix indices
    std::vector<unsigned int> t_ind;
    t_ind.reserve(t_frames.size());
    BOOST_FOREACH(const frame_id_t& fid, t_frames)
    {
      std::map<frame_id_t, unsigned int>::const_iterator fmi = frame_map.find(fid);
      // only add to the vector if in the map
      if( fmi != frame_map.end() )
      {
        t_ind.push_back(fmi->second);
      }
    }

    typedef std::vector<unsigned int>::const_iterator vitr_t;
    for( vitr_t tfi1 = t_ind.begin(); tfi1 != t_ind.end(); ++tfi1)
    {
      ++mm(*tfi1, *tfi1);
      vitr_t tfi2 = tfi1;
      ++tfi2;
      for( ; tfi2 != t_ind.end(); ++tfi2)
      {
        ++mm(*tfi1, *tfi2);
        ++mm(*tfi2, *tfi1);
      }
    }
  }
  return mm;
}



} // end namespace maptk
