/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include "match_features.h"

#include <vector>
#include <maptk/viscl/feature_set.h>
#include <maptk/viscl/descriptor_set.h>
#include <maptk/viscl/match_set.h>

#include <viscl/tasks/track_descr_match.h>
#include "utils.h"

namespace maptk
{

namespace vcl
{


/// Private implementation class
class match_features::priv
{
public:
  /// Constructor
  priv() : search_radius(200)
  {
  }

  // Copy Constructor
  priv(const priv& other) : search_radius(other.search_radius)
  {
  }

  viscl::track_descr_match matcher;
  unsigned int search_radius;
};


/// Constructor
match_features
::match_features()
: d_(new priv)
{
}


/// Copy Constructor
match_features
::match_features(const match_features& other)
: d_(new priv(*other.d_))
{
}


/// Destructor
match_features
::~match_features()
{
}

/// Get this algorithm's \link maptk::config_block configuration block \endlink
config_block_sptr
match_features
::get_configuration() const
{
  config_block_sptr config = algorithm::get_configuration();
  config->set_value("search_box_radius", d_->search_radius,
                    "Maximum pixel radius to search for kpt match.");
  return config;
}

/// Set this algorithm's properties via a config block
void
match_features
::set_configuration(config_block_sptr config)
{
  unsigned int sbr = config->get_value<unsigned int>("search_box_radius",
                                                     d_->search_radius);
  d_->matcher.set_search_box_radius(sbr);
}

/// Check that the algorithm's configuration config_block is valid
bool
match_features
::check_configuration(config_block_sptr config) const
{
  return true;
}

/// Match one set of features and corresponding descriptors to another
match_set_sptr
match_features
::match(feature_set_sptr feat1, descriptor_set_sptr desc1,
        feature_set_sptr feat2, descriptor_set_sptr desc2) const
{
  if( !desc1 || !desc2 )
  {
    return match_set_sptr();
  }

  viscl::buffer d1 = descriptors_to_viscl(*desc1);
  viscl::buffer d2 = descriptors_to_viscl(*desc2);

  vcl::feature_set::type f1 = vcl::features_to_viscl(*feat1);
  vcl::feature_set::type f2 = vcl::features_to_viscl(*feat2);

  size_t numkpts2 = feat2->size();
  viscl::buffer matches = d_->matcher.match(f1.features_, f1.kptmap_, d1,
                                            f2.features_, numkpts2, f2.kptmap_, d2);

  return match_set_sptr(new match_set(matches));
}


} // end namespace vcl

} // end namespace maptk
