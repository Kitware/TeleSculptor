/*ckwg +29
 * Copyright 2014-2015 by Kitware, Inc.
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
 * \brief Implementation of \link maptk::algo::close_loops_bad_frames_only
 *        close_loops_bad_frames_only \endlink
 */

#include "close_loops_bad_frames_only.h"

#include <algorithm>
#include <iostream>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <boost/algorithm/string/join.hpp>
#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include <boost/iterator/counting_iterator.hpp>

#include <vital/algo/algorithm.h>
#include <vital/exceptions/algorithm.h>


namespace kwiver {
namespace maptk {

namespace core
{

using namespace kwiver::vital;

/// Default Constructor
close_loops_bad_frames_only
::close_loops_bad_frames_only()
: enabled_(true),
  percent_match_req_(0.35),
  new_shot_length_(2),
  max_search_length_(5)
{
}


/// Copy Constructor
close_loops_bad_frames_only
::close_loops_bad_frames_only(const close_loops_bad_frames_only& other)
: enabled_(other.enabled_),
  percent_match_req_(other.percent_match_req_),
  new_shot_length_(other.new_shot_length_),
  max_search_length_(other.max_search_length_),
  matcher_(!other.matcher_ ? algo::match_features_sptr() : other.matcher_->clone())
{
}


std::string
close_loops_bad_frames_only
::description() const
{
  return "Attempts short-term loop closure based on percentage of feature "
    "points tracked.";
}


/// Get this alg's \link vital::config_block configuration block \endlink
  vital::config_block_sptr
close_loops_bad_frames_only
::get_configuration() const
{
  // get base config from base class
  vital::config_block_sptr config = algorithm::get_configuration();

  // Sub-algorithm implementation name + sub_config block
  // - Feature Matcher algorithm
  algo::match_features::get_nested_algo_configuration("feature_matcher", config, matcher_);

  // Bad frame detection parameters
  config->set_value("enabled", enabled_,
                    "Should bad frame detection be enabled? This option will attempt to "
                    "bridge the gap between frames which don't meet certain criteria "
                    "(percentage of feature points tracked) and will instead attempt "
                    "to match features on the current frame against past frames to "
                    "meet this criteria. This is useful when there can be bad frames.");

  config->set_value("percent_match_req", percent_match_req_,
                    "The required percentage of features needed to be matched for a "
                    "stitch to be considered successful (value must be between 0.0 and "
                    "1.0).");

  config->set_value("new_shot_length", new_shot_length_,
                    "Number of frames for a new shot to be considered valid before "
                    "attempting to stitch to prior shots.");

  config->set_value("max_search_length", max_search_length_,
                    "Maximum number of frames to search in the past for matching to "
                    "the end of the last shot.");

  return config;
}


/// Set this algo's properties via a config block
void
close_loops_bad_frames_only
::set_configuration(vital::config_block_sptr in_config)
{
  // Starting with our generated config_block to ensure that assumed values are present
  // An alternative is to check for key presence before performing a get_value() call.
  vital::config_block_sptr config = this->get_configuration();
  config->merge_config(in_config);

  // Setting nested algorithm instances via setter methods instead of directly
  // assigning to instance property.
  algo::match_features_sptr mf;
  algo::match_features::set_nested_algo_configuration("feature_matcher", config, mf);
  matcher_ = mf;

  // Settings for bad frame detection
  enabled_ = config->get_value<bool>("enabled");
  percent_match_req_ = config->get_value<double>("percent_match_req");
  max_search_length_ = config->get_value<unsigned>("max_search_length");
  new_shot_length_ = config->get_value<unsigned>("new_shot_length");
  new_shot_length_ = ( new_shot_length_ ? new_shot_length_ : 1 );
}


bool
close_loops_bad_frames_only
::check_configuration(vital::config_block_sptr config) const
{
  return (
    algo::match_features::check_nested_algo_configuration("feature_matcher", config)
    &&
    std::abs( config->get_value<double>("percent_match_req") ) <= 1.0
  );
}


/// Functor to help remove tracks from vector
bool track_id_in_set( track_sptr trk_ptr, std::set<track_id_t>* set_ptr )
{
  return set_ptr->find( trk_ptr->id() ) != set_ptr->end();
}


/// Handle track bad frame detection if enabled
vital::track_set_sptr
close_loops_bad_frames_only
::stitch( vital::frame_id_t frame_number,
          vital::track_set_sptr input,
          vital::image_container_sptr,
          vital::image_container_sptr ) const
{
  // check if enabled and possible
  if( !enabled_ || frame_number <= new_shot_length_ )
  {
    return input;
  }

  // check if we should attempt to stitch together past frames
  std::vector< vital::track_sptr > all_tracks = input->tracks();
  vital::frame_id_t frame_to_stitch = frame_number - new_shot_length_ + 1;
  double pt = input->percentage_tracked( frame_to_stitch - 1, frame_to_stitch );
  bool stitch_required = ( pt < percent_match_req_ );

  // confirm that the new valid shot criteria length is satisfied
  vital::frame_id_t frame_to_test = frame_to_stitch + 1;
  while( stitch_required && frame_to_test <= frame_number )
  {
    pt = input->percentage_tracked( frame_to_test - 1, frame_to_test );
    stitch_required = ( pt >= percent_match_req_ );
    frame_to_test++;
  }

  // determine if a stitch can be attempted
  if( !stitch_required )
  {
    return input;
  }

  // attempt to stitch start of shot frame against past n frames
  frame_to_test = frame_to_stitch - 2;
  vital::frame_id_t last_frame_to_test = 0;

  if( frame_to_test > max_search_length_ )
  {
    last_frame_to_test = frame_to_test - max_search_length_;
  }

  vital::track_set_sptr stitch_frame_set = input->active_tracks( frame_to_stitch );

  for( ; frame_to_test > last_frame_to_test; frame_to_test-- )
  {
    vital::track_set_sptr test_frame_set = input->active_tracks( frame_to_test );

    // run matcher alg
    vital::match_set_sptr mset = matcher_->match(test_frame_set->frame_features( frame_to_test ),
                                          test_frame_set->frame_descriptors( frame_to_test ),
                                          stitch_frame_set->frame_features( frame_to_stitch ),
                                          stitch_frame_set->frame_descriptors( frame_to_stitch ));

    // test matcher results
    unsigned total_features = static_cast<unsigned>(test_frame_set->size() + stitch_frame_set->size());

    if( 2*mset->size() >= static_cast<unsigned>(percent_match_req_*total_features) )
    {
      // modify track history and exit
      std::vector<vital::track_sptr> test_frame_trks = test_frame_set->tracks();
      std::vector<vital::track_sptr> stitch_frame_trks = stitch_frame_set->tracks();
      std::vector<vital::match> matches = mset->matches();
      std::set<vital::track_id_t> to_remove;

      for( unsigned i = 0; i < matches.size(); i++ )
      {
        if( test_frame_trks[ matches[i].first ]->append( *stitch_frame_trks[ matches[i].second ] ) )
        {
          to_remove.insert( stitch_frame_trks[ matches[i].second ]->id() );
        }
      }

      if( !to_remove.empty() )
      {
        all_tracks.erase(
          std::remove_if( all_tracks.begin(), all_tracks.end(), boost::bind( track_id_in_set, _1, &to_remove ) ),
          all_tracks.end()
        );
      }

      return track_set_sptr( new simple_track_set( all_tracks ) );
    }
  }

  // bad frame detection has failed
  return input;
}


} // end namespace core

} // end namespace maptk
} // end namespace kwiver
