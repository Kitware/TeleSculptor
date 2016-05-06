/*ckwg +29
 * Copyright 2016 by Kitware, Inc.
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
 * \brief Implementation of close_loops_keyframe
 */

#include "close_loops_keyframe.h"

#include <set>
#include <string>
#include <vector>
#include <functional>

#include <vital/exceptions/algorithm.h>
#include <vital/algo/match_features.h>


namespace kwiver {
namespace maptk {
namespace core {

using namespace kwiver::vital;


/// Functor to help remove tracks from vector
bool track_in_set( track_sptr trk_ptr, std::set<track_id_t>* set_ptr )
{
  return set_ptr->find( trk_ptr->id() ) != set_ptr->end();
}


/// Private implementation class
class close_loops_keyframe::priv
{
public:
  /// Constructor
  priv()
    : match_req(100),
      search_bandwidth(10),
      min_keyframe_misses(5),
      m_logger( vital::get_logger( "close_loops_keyframe" ))
  {
  }

  priv(const priv& other)
    : match_req(other.match_req),
      search_bandwidth(other.search_bandwidth),
      min_keyframe_misses(other.min_keyframe_misses),
      matcher(!other.matcher ? algo::match_features_sptr() : other.matcher->clone()),
      m_logger( vital::get_logger( "close_loops_keyframe" ))
  {
  }

  /// Stich the current frame to the specified target frame number
  track_set_sptr
  stitch( frame_id_t target_frame,
          track_set_sptr input,
          frame_id_t current_frame,
          std::vector<track_sptr>& current_tracks,
          feature_set_sptr current_features,
          descriptor_set_sptr current_descriptors,
          int& num_matched, int& num_linked) const
  {
    num_matched = num_linked = 0;
    // extract the subset of tracks on the target frame
    track_set_sptr tgt_trks = input->active_tracks(target_frame);
    // extract the set of features on the target frame
    feature_set_sptr target_features = tgt_trks->frame_features(target_frame);
    // extract the set of descriptor on the target frame
    descriptor_set_sptr target_descriptors = tgt_trks->frame_descriptors(target_frame);

    // run the matcher algorithm between the target and current frames
    match_set_sptr mset = this->matcher->match(target_features, target_descriptors,
                                               current_features, current_descriptors);

    num_matched = mset->size();
    if( num_matched < this->match_req )
    {
      return input;
    }

    // modify track history
    std::vector<vital::track_sptr> target_tracks = tgt_trks->tracks();
    std::vector<vital::match> matches = mset->matches();
    std::set<vital::track_id_t> to_remove;

    for( unsigned i = 0; i < matches.size(); i++ )
    {
      unsigned tgt_idx = matches[i].first;
      unsigned cur_idx = matches[i].second;
      if( target_tracks[ tgt_idx ]->append( *current_tracks[ cur_idx ] ) )
      {
        to_remove.insert( current_tracks[ cur_idx ]->id() );
        current_tracks[ cur_idx ] = target_tracks[ tgt_idx ];
      }
    }

    if( !to_remove.empty() )
    {
      num_linked = to_remove.size();
      std::vector<track_sptr> all_tracks = input->tracks();
      all_tracks.erase(
        std::remove_if( all_tracks.begin(), all_tracks.end(),
                        std::bind( track_in_set, std::placeholders::_1, &to_remove ) ),
        all_tracks.end()
      );
      // recreate the track set with the new filtered tracks
      input = std::make_shared<simple_track_set>( all_tracks );
    }
    return input;
  }

  /// number of feature matches required for acceptance
  int match_req;

  /// number of adjacent frames to match
  int search_bandwidth;

  /// minimum number of keyframe misses before creating a new keyframe
  int min_keyframe_misses;

  /// Indices of the the selected keyframes
  std::vector<frame_id_t> keyframes;

  /// histogram of matches associated with each frame
  std::map<frame_id_t, unsigned int> frame_matches;

  /// a collection of recent frame that didn't match any keyframe
  std::vector<frame_id_t> keyframe_misses;

  /// The feature matching algorithm to use
  vital::algo::match_features_sptr matcher;

  /// Logger handle
  vital::logger_handle_t m_logger;
};


/// Constructor
close_loops_keyframe
::close_loops_keyframe()
: d_(new priv)
{
}


/// Copy Constructor
close_loops_keyframe
::close_loops_keyframe(const close_loops_keyframe& other)
: d_(new priv(*other.d_))
{
}


/// Destructor
close_loops_keyframe
::~close_loops_keyframe() VITAL_NOTHROW
{
}


std::string
close_loops_keyframe
::description() const
{
  return "Establishes keyframes matches to all keyframes";
}


/// Get this alg's \link vital::config_block configuration block \endlink
  vital::config_block_sptr
close_loops_keyframe
::get_configuration() const
{
  // get base config from base class
  vital::config_block_sptr config = algorithm::get_configuration();

  // Sub-algorithm implementation name + sub_config block
  // - Feature Matcher algorithm
  algo::match_features::get_nested_algo_configuration("feature_matcher", config, d_->matcher);

  config->set_value("match_req", d_->match_req,
                    "The required number of features needed to be matched for a success.");

  config->set_value("search_bandwidth", d_->search_bandwidth,
                    "number of adjacent frames to match to (must be at least 1)");

  config->set_value("min_keyframe_misses", d_->min_keyframe_misses,
                    "minimum number of keyframe match misses before creating a new keyframe. "
                    "A match miss occures when the current frame does not match any existing "
                    "keyframe (must be at least 1)");

  return config;
}


/// Set this algo's properties via a config block
void
close_loops_keyframe
::set_configuration(vital::config_block_sptr in_config)
{
  // Starting with our generated config_block to ensure that assumed values are present
  // An alternative is to check for key presence before performing a get_value() call.
  vital::config_block_sptr config = this->get_configuration();
  config->merge_config(in_config);

  // Setting nested algorithm configuration
  algo::match_features::set_nested_algo_configuration("feature_matcher",
                                                      config, d_->matcher);

  d_->match_req = config->get_value<int>("match_req");
  d_->search_bandwidth = config->get_value<int>("search_bandwidth");
  d_->min_keyframe_misses = config->get_value<int>("min_keyframe_misses");
}


bool
close_loops_keyframe
::check_configuration(vital::config_block_sptr config) const
{

  return (
    algo::match_features::check_nested_algo_configuration("feature_matcher", config)
    && config->get_value<int>("search_bandwidth") >= 1
    && config->get_value<int>("min_keyframe_misses") >= 1
  );
}


/// Frame stitching using keyframe-base matching
vital::track_set_sptr
close_loops_keyframe
::stitch( vital::frame_id_t frame_number,
          vital::track_set_sptr input,
          vital::image_container_sptr,
          vital::image_container_sptr ) const
{
  // initialize frame matches for this frame
  d_->frame_matches[frame_number] = 0;

  // do nothing for the first two frames, there is nothing to match
  if( frame_number < 2 )
  {
    return input;
  }

  // compute the last frame we need to match to within the search bandwidth
  // the conditional accounts for the boundary case at startup
  frame_id_t last_frame = 0;
  if(frame_number > d_->search_bandwidth)
  {
    last_frame = frame_number - d_->search_bandwidth;
  }

  // the first frame is always a key frame (for now)
  // This could proably be improved
  if(d_->keyframes.empty())
  {
    d_->keyframes.push_back(input->first_frame());
  }

  // extract the subset of tracks on the current frame and their
  // features and descriptors
  vital::track_set_sptr current_set = input->active_tracks( frame_number );
  std::vector<vital::track_sptr> current_tracks = current_set->tracks();
  vital::descriptor_set_sptr current_descriptors =
      current_set->frame_descriptors( frame_number );
  vital::feature_set_sptr current_features =
      current_set->frame_features( frame_number );

  // number of matched features and linked tracks are returned by reference
  // in these variables
  int num_matched = 0, num_linked = 0;
  // used to compute the maximum number of matches between the current frame
  // and any of the key frames
  int max_keyframe_matched = 0;

  // use this iterator to step backward through the keyframes
  // as we step backward through the neighborhood to identify
  // which neighborhood frames are also keyframes
  auto kitr = d_->keyframes.rbegin();
  // since loop closure starts at frame n-2, if the latest
  // keyframe happens to be n-1 we need to skip that one
  if (*kitr == frame_number-1)
  {
    ++kitr;
  }

  // stitch with all frames within a neighborhood of the current frame
  for(vital::frame_id_t f = frame_number - 2; f >= last_frame; f-- )
  {
    input = d_->stitch(f, input, frame_number, current_tracks,
                       current_features, current_descriptors,
                       num_matched, num_linked);
    // accumulate matches to help assign keyframes later
    d_->frame_matches[frame_number] += num_matched;

    // keyframes can occur within the current search neighborhood
    // if this frame is a keyframe then account for it in the
    // computation of the maximum number of matches to all keyframes
    std::string frame_name = "";
    if(kitr != d_->keyframes.rend() && f == *kitr)
    {
      if( num_matched > max_keyframe_matched )
      {
        max_keyframe_matched = num_matched;
      }
      ++kitr;
      frame_name = "keyframe ";
    }
    LOG_INFO(d_->m_logger, "Matching frame " << frame_number << " to "
                           << frame_name << f
                           << " has "<< num_matched << " matches and "
                           << num_linked << " joined tracks");
  }

  // stitch with all previous keyframes
  for(auto kitr = d_->keyframes.rbegin(); kitr != d_->keyframes.rend(); ++kitr)
  {
    // if this frame was already matched above then skip it
    if(*kitr >= last_frame)
    {
      continue;
    }
    input = d_->stitch(*kitr, input, frame_number, current_tracks,
                       current_features, current_descriptors,
                       num_matched, num_linked);
    LOG_INFO(d_->m_logger, "Matching frame " << frame_number << " to keyframe "<< *kitr
                           << " has "<< num_matched << " matches and "
                           << num_linked << " joined tracks");
    if( num_matched > max_keyframe_matched )
    {
      max_keyframe_matched = num_matched;
    }
  }

  // keep track of frames that matched no keyframes
  if (max_keyframe_matched < d_->match_req)
  {
    d_->keyframe_misses.push_back(frame_number);
    LOG_DEBUG(d_->m_logger, "Frame " << frame_number << " added to keyframe misses. "
                            << "Count: "<<d_->keyframe_misses.size());
  }

  // If we've seen enough keyframe misses and the first miss has passed outside
  // of the search bandwidth, then add a new key frame by selecting the frame
  // since the first miss that has been most successful at matching.
  if (d_->keyframe_misses.size() > d_->min_keyframe_misses &&
      d_->keyframe_misses.front() < last_frame)
  {
    auto hitr = d_->frame_matches.find(d_->keyframe_misses.front());
    unsigned int max_matches = 0;
    frame_id_t max_id = 0;
    // find the frame with the most accumulated matches
    for(++hitr; hitr != d_->frame_matches.end(); ++hitr)
    {
      if(hitr->second > max_matches)
      {
        max_matches = hitr->second;
        max_id = hitr->first;
      }
    }
    // create the new keyframe and clear the list of misses
    LOG_INFO(d_->m_logger, "creating new keyframe on frame " << max_id);
    d_->keyframes.push_back(max_id);
    d_->keyframe_misses.clear();
  }

  return input;
}


} // end namespace core
} // end namespace maptk
} // end namespace kwiver
