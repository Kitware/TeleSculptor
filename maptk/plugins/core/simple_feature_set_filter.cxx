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

#include "simple_feature_set_filter.h"

#include <algorithm>
#include <boost/make_shared.hpp>


namespace maptk
{

namespace core
{


bool feature_less(feature_sptr l, feature_sptr r)
{
  return l->magnitude() < r->magnitude();
}

/// Private implementation class
class simple_feature_set_filter::priv
{
public:
  /// Constructor
  priv() :
    top_percent(0.2),
    min_features(100)
  {
  }

  priv(const priv& other) :
    top_percent(other.top_percent),
    min_features(other.min_features)
  {
  }

  void
  filter(feature_set_sptr feat, std::vector<feature_sptr> &filtered) const
  {
    filtered.clear();
    std::vector<feature_sptr> sorted(feat->features().begin(), feat->features().end());
    std::sort(sorted.begin(), sorted.end(), feature_less);

    unsigned int cutoff = std::max(min_features, static_cast<unsigned int>(top_percent * sorted.size()));

    filtered.resize(cutoff);
    for (unsigned int i = 0; i < cutoff; i++)
    {
      filtered[i] = sorted[i];
    }
  }

  double top_percent;
  unsigned int min_features;
};


/// Constructor
simple_feature_set_filter
::simple_feature_set_filter()
: d_(new priv)
{
}


/// Copy Constructor
simple_feature_set_filter
::simple_feature_set_filter(const simple_feature_set_filter& other)
: d_(new priv(*other.d_))
{
}


/// Destructor
simple_feature_set_filter
::~simple_feature_set_filter()
{
}


/// Get this algorithm's \link maptk::config_block configuration block \endlink
config_block_sptr
simple_feature_set_filter
::get_configuration() const
{
  // get base config from base class
  config_block_sptr config =
      maptk::algo::feature_set_filter::get_configuration();

  config->set_value("top_percent", d_->top_percent,
                    "Percent of strongest keypoints to keep, range (0.0, 1.0]");

  config->set_value("min_features", d_->min_features,
                    "minimum number of features to keep");

  return config;
}


/// Set this algorithm's properties via a config block
void
simple_feature_set_filter
::set_configuration(config_block_sptr config)
{
  d_->top_percent = config->get_value<double>("top_percent", d_->top_percent);
  d_->min_features = config->get_value<unsigned int>("min_features", d_->min_features);
}


/// Check that the algorithm's configuration config_block is valid
bool
simple_feature_set_filter
::check_configuration(config_block_sptr config) const
{
  return true;
}


/// Match one set of features and corresponding descriptors to another
feature_set_sptr
simple_feature_set_filter
::filter(feature_set_sptr feat) const
{
  std::vector<feature_sptr> filtered;
  d_->filter(feat, filtered);

  return boost::make_shared<maptk::simple_feature_set>(maptk::simple_feature_set(filtered));
}

} // end namespace core

} // end namespace maptk
