/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include <maptk/core/algo/track_features.h>
#include <maptk/core/algo/algorithm.txx>
#include <maptk/core/exceptions/algorithm.h>

#include <algorithm>
#include <iostream>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <boost/foreach.hpp>
#include <boost/iterator/counting_iterator.hpp>
#include <boost/algorithm/string/join.hpp>

/**
 * \file
 * \brief Implementation of \link maptk::algo::track_features
          track_features \endlink functions
 */



INSTANTIATE_ALGORITHM_DEF(maptk::algo::track_features);


namespace maptk
{

namespace algo
{

/// Get this alg's \link maptk::config_block configuration block \endlink
config_block_sptr
track_features
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

  return config;
}

/// Set this algo's properties via a config block
void
track_features
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
}

bool
track_features
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

} // end namespace algo

} // end namespace maptk
