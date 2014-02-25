/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief Implementation of \link maptk::algo::track_features_default
 *        track_features_default \endlink
 */

#include <maptk/core/algo/track_features_default.h>
#include <maptk/core/algo/algorithm.txx>
#include <maptk/core/exceptions/algorithm.h>

#include <algorithm>
#include <iostream>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include <boost/iterator/counting_iterator.hpp>
#include <boost/algorithm/string/join.hpp>


namespace maptk
{

namespace algo
{


/// Default Constructor
track_features_default
::track_features_default()
: next_track_id_(0),
  stitching_enabled_(false),
  stitching_percent_match_req_(0.4),
  stitching_new_shot_length_(2),
  stitching_max_search_length_(5)
{
}


/// Copy Constructor
track_features_default
::track_features_default(const track_features_default& other)
: next_track_id_(other.next_track_id_)
{
}


/// Get this alg's \link maptk::config_block configuration block \endlink
config_block_sptr
track_features_default
::get_configuration() const
{
  // get base config from base class
  config_block_sptr config = algorithm::get_configuration();

  // Sub-algorithm implementation name + sub_config block
  // - Feature Detector algorithm
  detect_features::get_nested_algo_configuration("feature_detector", config, detector_);

  // - Descriptor Extractor algorithm
  extract_descriptors::get_nested_algo_configuration("descriptor_extractor", config, extractor_);

  // - Feature Matcher algorithm
  match_features::get_nested_algo_configuration("feature_matcher", config, matcher_);

  // Frame stitching parameters
  config->set_value("stitching_enabled", stitching_enabled_,
                    "Should frame stitching be enabled? This option will attempt to "
                    "bridge the gap between frames which don't meet certain criteria "
                    "(percentage of feature points tracked) and will instead attempt "
                    "to match features on the current frame against past frames to "
                    "meet this criteria. This is useful when there can be bad frames.");

  config->set_value("stitching_percent_match_req", stitching_percent_match_req_,
                    "The required percentage of features needed to be matched for a "
                    "stitch to be considered successful (value must be between 0.0 and "
                    "1.0).");

  config->set_value("stitching_new_shot_length", stitching_new_shot_length_,
                    "Number of frames for a new shot to be considered valid before "
                    "attempting to stitch to prior shots.");

  config->set_value("stitching_max_search_length", stitching_max_search_length_,
                    "Maximum number of frames to search in the past for matching to "
                    "the end of the last shot.");

  return config;
}


/// Set this algo's properties via a config block
void
track_features_default
::set_configuration(config_block_sptr in_config)
{
  // Starting with our generated config_block to ensure that assumed values are present
  // An alternative is to check for key presence before performing a get_value() call.
  config_block_sptr config = this->get_configuration();
  config->merge_config(in_config);

  // Setting nested algorithm instances via setter methods instead of directly
  // assigning to instance property.
  detect_features_sptr df;
  detect_features::set_nested_algo_configuration("feature_detector", config, df);
  detector_ = df;

  extract_descriptors_sptr ed;
  extract_descriptors::set_nested_algo_configuration("descriptor_extractor", config, ed);
  extractor_ = ed;

  match_features_sptr mf;
  match_features::set_nested_algo_configuration("feature_matcher", config, mf);
  matcher_ = mf;

  // Settings for frame stitching
  stitching_enabled_ = config->get_value<bool>("stitching_enabled");

  if( stitching_enabled_ )
  {
    stitching_percent_match_req_ = config->get_value<double>("stitching_percent_match_req");
    stitching_max_search_length_ = config->get_value<unsigned>("stitching_max_search_length");
    stitching_new_shot_length_ = config->get_value<unsigned>("stitching_new_shot_length");
    stitching_new_shot_length_ = ( stitching_new_shot_length_ ? stitching_new_shot_length_ : 1 );
  }
}


bool
track_features_default
::check_configuration(config_block_sptr config) const
{
  return (
    detect_features::check_nested_algo_configuration("feature_detector", config)
    &&
    extract_descriptors::check_nested_algo_configuration("descriptor_extractor", config)
    &&
    match_features::check_nested_algo_configuration("feature_matcher", config)
  );
}


/// Extend a previous set of tracks using the current frame
track_set_sptr
track_features_default
::track(track_set_sptr prev_tracks,
        unsigned int frame_number,
        image_container_sptr image_data) const
{
  // verify that all dependent algorithms have been initialized
  if( !detector_ || !extractor_ || !matcher_ )
  {
    return track_set_sptr();
  }

  // detect features on the current frame
  feature_set_sptr curr_feat = detector_->detect(image_data);

  // extract descriptors on the current frame
  descriptor_set_sptr curr_desc = extractor_->extract(image_data, curr_feat);

  std::vector<feature_sptr> vf = curr_feat->features();
  std::vector<descriptor_sptr> df = curr_desc->descriptors();

  // special case for the first frame
  if( !prev_tracks )
  {
    typedef std::vector<feature_sptr>::const_iterator feat_itr;
    typedef std::vector<descriptor_sptr>::const_iterator desc_itr;
    feat_itr fit = vf.begin();
    desc_itr dit = df.begin();
    std::vector<track_sptr> new_tracks;
    for(; fit != vf.end() && dit != df.end(); ++fit, ++dit)
    {
       track::track_state ts(frame_number, *fit, *dit);
       new_tracks.push_back(track_sptr(new maptk::track(ts)));
       new_tracks.back()->set_id(this->next_track_id_++);
    }
    return track_set_sptr(new simple_track_set(new_tracks));
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
    all_tracks.push_back(track_sptr(new maptk::track(ts)));
    all_tracks.back()->set_id(this->next_track_id_++);
  }

  return track_stitching(frame_number, track_set_sptr(new simple_track_set(all_tracks)));
}


/// Functor to help remove tracks from vector
bool track_id_in_set( track_sptr trk_ptr, std::set<track_id_t>* set_ptr )
{
  return set_ptr->find( trk_ptr->id() ) != set_ptr->end();
}

/// Handle track stitching if enabled
track_set_sptr
track_features_default
::track_stitching( frame_id_t frame_number, track_set_sptr input ) const
{
  // check if enabled and possible
  if( !stitching_enabled_ || frame_number > stitching_new_shot_length_ )
  {
    return input;
  }

  // check if we should attempt to stitch together past frames
  std::vector< track_sptr > all_tracks = input->tracks();
  frame_id_t frame_to_stitch = frame_number - stitching_new_shot_length_ + 1;
  double pt = input->percentage_tracked( frame_to_stitch - 1, frame_to_stitch );
  bool stitch_required = ( pt < stitching_percent_match_req_ );

  // confirm that the new valid shot criteria length is satisfied
  frame_id_t frame_to_test = frame_to_stitch + 1;
  while( stitch_required && frame_to_test <= frame_number )
  {
    pt = input->percentage_tracked( frame_to_test - 1, frame_to_test );
    stitch_required = ( pt >= stitching_percent_match_req_ );
  }

  // determine if a stitch can be attempted
  if( !stitch_required )
  {
    return input;
  }

  // attempt to stitch start of shot frame against past n frames
  frame_to_test = frame_to_stitch - 2;
  frame_id_t last_frame_to_test = 0;

  if( frame_to_test > stitching_max_search_length_ )
  {
    last_frame_to_test = frame_to_test - stitching_max_search_length_;
  }

  track_set_sptr stitch_frame_set = input->active_tracks( frame_to_stitch );

  for( ; frame_to_test > last_frame_to_test; frame_to_test-- )
  {
    track_set_sptr test_frame_set = input->active_tracks( frame_to_test );

    // run matcher alg
    match_set_sptr mset = matcher_->match(test_frame_set->frame_features( frame_to_test ),
                                          test_frame_set->frame_descriptors( frame_to_test ),
                                          stitch_frame_set->frame_features( frame_to_stitch ),
                                          stitch_frame_set->frame_descriptors( frame_to_stitch ));

    // test matcher results
    unsigned total_features = test_frame_set->size() + stitch_frame_set->size();

    if( 2*mset->size() >= static_cast<unsigned>(stitching_percent_match_req_*total_features) )
    {
      // modify track history and exit
      std::vector<track_sptr> test_frame_trks = test_frame_set->tracks();
      std::vector<track_sptr> stitch_frame_trks = stitch_frame_set->tracks();
      std::vector<match> matches = mset->matches();
      std::set<track_id_t> to_remove;

      for( unsigned i = 0; i < matches.size(); i++ )
      {
        if( test_frame_trks[ matches[i].first ]->append( *stitch_frame_trks[ matches[i].second ] ) )
        {
          to_remove.insert( stitch_frame_trks[ matches[i].second ]->id() );
        }
      }

      if( !to_remove.empty() )
      {
        std::remove_if( all_tracks.begin(), all_tracks.end(),
                        boost::bind( track_id_in_set, _1, &to_remove ) );
      }

      return track_set_sptr( new simple_track_set( all_tracks ) );
    }
  }

  // stitching has failed
  return input;
}


} // end namespace algo

} // end namespace maptk
