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

#include "match_features.h"

#include <vector>

#include <maptk/plugins/viscl/descriptor_set.h>
#include <maptk/plugins/viscl/feature_set.h>
#include <maptk/plugins/viscl/match_set.h>
#include <maptk/plugins/viscl/utils.h>

#include <viscl/tasks/track_descr_match.h>


namespace kwiver {
namespace maptk {

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

/// Get this algorithm's \link kwiver::vital::config_block configuration block \endlink
kwiver::vital::config_block_sptr
match_features
::get_configuration() const
{
  kwiver::vital::config_block_sptr config = algorithm::get_configuration();
  config->set_value("search_box_radius", d_->search_radius,
                    "Maximum pixel radius to search for kpt match.");
  return config;
}

/// Set this algorithm's properties via a config block
void
match_features
::set_configuration(kwiver::vital::config_block_sptr config)
{
  unsigned int sbr = config->get_value<unsigned int>("search_box_radius",
                                                     d_->search_radius);
  d_->matcher.set_search_box_radius(sbr);
}

/// Check that the algorithm's configuration kwiver::vital::config_block is valid
bool
match_features
::check_configuration(kwiver::vital::config_block_sptr config) const
{
  return true;
}

/// Match one set of features and corresponding descriptors to another
kwiver::vital::match_set_sptr
match_features
::match(kwiver::vital::feature_set_sptr feat1, kwiver::vital::descriptor_set_sptr desc1,
        kwiver::vital::feature_set_sptr feat2, kwiver::vital::descriptor_set_sptr desc2) const
{
  if( !desc1 || !desc2 )
  {
    return kwiver::vital::match_set_sptr();
  }

  viscl::buffer d1 = descriptors_to_viscl(*desc1);
  viscl::buffer d2 = descriptors_to_viscl(*desc2);

  vcl::feature_set::type f1 = vcl::features_to_viscl(*feat1);
  vcl::feature_set::type f2 = vcl::features_to_viscl(*feat2);

  size_t numkpts2 = feat2->size();
  viscl::buffer matches = d_->matcher.match(f1.features_, f1.kptmap_, d1,
                                            f2.features_, numkpts2, f2.kptmap_, d2);

  return kwiver::vital::match_set_sptr(new match_set(matches));
}


} // end namespace vcl

} // end namespace maptk
} // end namespace kwiver
