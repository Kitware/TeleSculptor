/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include <maptk/core/algo/track_features.h>
#include <maptk/core/algo/algorithm.txx>
#include <maptk/core/exceptions/algorithm.h>

#include <algorithm>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <boost/foreach.hpp>
#include <boost/iterator/counting_iterator.hpp>
#include <boost/algorithm/string/join.hpp>


INSTANTIATE_ALGORITHM_DEF(maptk::algo::track_features);


namespace maptk
{

namespace algo
{

/// Get this alg's \link maptk::config_block configuration block \endlink
config_block_sptr
track_features
::get_configuration()
{
  config_block_sptr config = algorithm::get_configuration();

  // Value to assigned is either the impl name of the currently assigned algo
  // or a listing of all available impl types (when none is currently assigned)
# define rnames(algo) boost::algorithm::join(algo::registered_names(), " | ")
  std::string
    df_impl = detector_  ? detector_->impl_name()  : rnames(detect_features)
  , ed_impl = extractor_ ? extractor_->impl_name() : rnames(extract_descriptors)
  , mf_impl = matcher_   ? matcher_->impl_name()   : rnames(match_features)
  ;
# undef rnames

  config->set_value("detect_features_algo", df_impl);
  config->set_value("extract_descriptors_algo", ed_impl);
  config->set_value("match_features_algo", mf_impl);

  // merge in sub-algorithm configs if there is anything in their blocks
  // if( // TODO:: RIGHT HERE!!! THIS THING!!!

  return config;
}

/// Set this algo's properties via a config block
void
track_features
::configure(config_block_sptr config)
{
  std::string
    df_impl_name = config->get_value<std::string>("detect_features_algo"),
    ed_impl_name = config->get_value<std::string>("extract_descriptors_algo"),
    mf_impl_name = config->get_value<std::string>("match_features_algo");

  // Check that value read in for each algorithm impl is value for that algo type
  std::vector<std::string>
    valid_df_names = detect_features::registered_names(),
    valid_ed_names = extract_descriptors::registered_names(),
    valid_mf_names = match_features::registered_names();

# define check_valid(algo_name, name, check_vector)                                     \
  do                                                                                    \
  {                                                                                     \
    if(std::find(check_vector.begin(), check_vector.end(), name) == check_vector.end()) \
    {                                                                                   \
      std::ostringstream sstr;                                                          \
      sstr << "Invalid algorithm impl name given for " << algo_name << ". "             \
           << "Given: \"" << name << "\"";                                              \
      throw algorithm_configuration_exception(type_name(), impl_name(), sstr.str());    \
    }                                                                                   \
  } while(false)

  check_valid("detect_features_algo",     df_impl_name, valid_df_names);
  check_valid("extract_descriptors_algo", ed_impl_name, valid_ed_names);
  check_valid("match_features_algo",      mf_impl_name, valid_mf_names);
# undef check_valid

  // Setting algorithm impls to member variables
  this->set_feature_detector(detect_features::create(df_impl_name));
  this->set_descriptor_extractor(extract_descriptors::create(ed_impl_name));
  this->set_feature_matcher(match_features::create(mf_impl_name));
}

/// Extend a previous set of tracks using the current frame
track_set_sptr
simple_track_features
::track(track_set_sptr prev_tracks,
        unsigned int frame_number,
        image_container_sptr image_data) const
{
  // verify that all dependent algorithms have been initialized
  if( ! detector_ || ! extractor_ || !matcher_ )
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
                      boost::counting_iterator<unsigned>(vf.size()),
                      matched.begin(), matched.end(),
                      unmatched_insert_itr);
  BOOST_FOREACH(unsigned i, unmatched)
  {
    track::track_state ts(frame_number, vf[i], df[i]);
    all_tracks.push_back(track_sptr(new maptk::track(ts)));
  }
  return track_set_sptr(new simple_track_set(all_tracks));
}


} // end namespace algo

} // end namespace maptk
