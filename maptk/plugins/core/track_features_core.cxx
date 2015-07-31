/*ckwg +29
 * Copyright 2013-2015 by Kitware, Inc.
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
 * \brief Implementation of \link maptk::algo::track_features_core
 *        track_features_core \endlink
 */

#include "track_features_core.h"

#include <algorithm>
#include <iostream>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <boost/algorithm/string/join.hpp>
#include <boost/foreach.hpp>
#include <boost/iterator/counting_iterator.hpp>

#include <vital/algo/algorithm.h>
#include <vital/exceptions/algorithm.h>
#include <vital/exceptions/image.h>

using namespace kwiver::vital;

namespace kwiver {
namespace maptk {

namespace core
{


/// Default Constructor
track_features_core
::track_features_core()
: next_track_id_(0)
{
}


/// Copy Constructor
track_features_core
::track_features_core(const track_features_core& other)
: next_track_id_(other.next_track_id_)
{
}


/// Get this alg's \link kwiver::vital::config_block configuration block \endlink
kwiver::vital::config_block_sptr
track_features_core
::get_configuration() const
{
  // get base config from base class
  kwiver::vital::config_block_sptr config = algorithm::get_configuration();

  // Sub-algorithm implementation name + sub_config block
  // - Feature Detector algorithm
  algo::detect_features::get_nested_algo_configuration("feature_detector", config, detector_);

  // - Descriptor Extractor algorithm
  algo::extract_descriptors::get_nested_algo_configuration("descriptor_extractor", config, extractor_);

  // - Feature Matcher algorithm
  algo::match_features::get_nested_algo_configuration("feature_matcher", config, matcher_);

  // - Loop closure algorithm
  algo::close_loops::get_nested_algo_configuration("loop_closer", config, closer_);

  return config;
}


/// Set this algo's properties via a config block
void
track_features_core
::set_configuration(kwiver::vital::config_block_sptr in_config)
{
  // Starting with our generated config_block to ensure that assumed values are present
  // An alternative is to check for key presence before performing a get_value() call.
  kwiver::vital::config_block_sptr config = this->get_configuration();
  config->merge_config(in_config);

  // Setting nested algorithm instances via setter methods instead of directly
  // assigning to instance property.
  algo::detect_features_sptr df;
  algo::detect_features::set_nested_algo_configuration("feature_detector", config, df);
  detector_ = df;

  algo::extract_descriptors_sptr ed;
  algo::extract_descriptors::set_nested_algo_configuration("descriptor_extractor", config, ed);
  extractor_ = ed;

  algo::match_features_sptr mf;
  algo::match_features::set_nested_algo_configuration("feature_matcher", config, mf);
  matcher_ = mf;

  algo::close_loops_sptr cl;
  algo::close_loops::set_nested_algo_configuration("loop_closer", config, cl);
  closer_ = cl;
}


bool
track_features_core
::check_configuration(kwiver::vital::config_block_sptr config) const
{
  return (
    algo::detect_features::check_nested_algo_configuration("feature_detector", config)
    &&
    algo::extract_descriptors::check_nested_algo_configuration("descriptor_extractor", config)
    &&
    algo::match_features::check_nested_algo_configuration("feature_matcher", config)
    &&
    algo::close_loops::check_nested_algo_configuration("loop_closer", config)
  );
}


/// Extend a previous set of tracks using the current frame
track_set_sptr
track_features_core
::track(track_set_sptr prev_tracks,
        unsigned int frame_number,
        image_container_sptr image_data,
        image_container_sptr mask) const
{
  // verify that all dependent algorithms have been initialized
  if( !detector_ || !extractor_ || !matcher_ || !closer_ )
  {
    // Something did not initialize, return an empty vector
    /// \todo Convert to log message
    std::cerr << "ERROR - not all dependent algorithms have been initialized.\n";
    return track_set_sptr();
  }

  // Check that the given mask, when non-zero, matches the size of the image
  // data provided
  if( mask && mask->size() > 0 &&
      ( image_data->width() != mask->width() ||
        image_data->height() != mask->height() ) )
  {
    throw image_size_mismatch_exception(
        "Core track feature algorithm given a non-zero mask image that is "
        "not the same shape as the provided image data.",
        image_data->width(), image_data->height(),
        mask->width(), mask->height()
        );
  }

  // detect features on the current frame
  feature_set_sptr curr_feat = detector_->detect(image_data, mask);

  // extract descriptors on the current frame
  descriptor_set_sptr curr_desc = extractor_->extract(image_data, curr_feat, mask);

  std::vector<feature_sptr> vf = curr_feat->features();
  std::vector<descriptor_sptr> df = curr_desc->descriptors();

  // special case for the first frame
  if( !prev_tracks )
  {
    typedef std::vector<feature_sptr>::const_iterator feat_itr;
    typedef std::vector<descriptor_sptr>::const_iterator desc_itr;
    feat_itr fit = vf.begin();
    desc_itr dit = df.begin();
    std::vector<kwiver::vital::track_sptr> new_tracks;
    for(; fit != vf.end() && dit != df.end(); ++fit, ++dit)
    {
       track::track_state ts(frame_number, *fit, *dit);
       new_tracks.push_back(kwiver::vital::track_sptr(new kwiver::vital::track(ts)));
       new_tracks.back()->set_id(this->next_track_id_++);
    }
    // call loop closure on the first frame to establish this
    // frame as the first frame for loop closing purposes
    return closer_->stitch(frame_number,
                           track_set_sptr(new simple_track_set(new_tracks)),
                           image_data, mask);
  }

  // match features to from the previous to the current frame
  match_set_sptr mset = matcher_->match(prev_tracks->last_frame_features(),
                                        prev_tracks->last_frame_descriptors(),
                                        curr_feat,
                                        curr_desc);

  track_set_sptr active_set = prev_tracks->active_tracks();
  std::vector<track_sptr> active_tracks = active_set->tracks();
  std::vector<track_sptr> all_tracks = prev_tracks->tracks();
  std::vector<match> vm = mset->matches();
  std::set<unsigned> matched;

  BOOST_FOREACH(match m, vm)
  {
    matched.insert(m.second);
    track_sptr t = active_tracks[m.first];
    track::track_state ts(frame_number, vf[m.second], df[m.second]);
    t->append(ts);
  }

  // find the set of unmatched active track indices
  std::vector<unsigned> unmatched;
  std::back_insert_iterator<std::vector<unsigned> > unmatched_insert_itr(unmatched);
  std::set_difference(boost::counting_iterator<unsigned>(0),
                      boost::counting_iterator<unsigned>(static_cast<unsigned int>(vf.size())),
                      matched.begin(), matched.end(),
                      unmatched_insert_itr);

  BOOST_FOREACH(unsigned i, unmatched)
  {
    track::track_state ts(frame_number, vf[i], df[i]);
    all_tracks.push_back(kwiver::vital::track_sptr(new kwiver::vital::track(ts)));
    all_tracks.back()->set_id(this->next_track_id_++);
  }

  track_set_sptr stitched_tracks = closer_->stitch(frame_number,
    track_set_sptr(new simple_track_set(all_tracks)),
    image_data, mask);

  return stitched_tracks;
}


} // end namespace core

} // end namespace maptk
} // end namespace kwiver
