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

#include "filter_features_magnitude.h"

#include <maptk/logging_macros.h>

#include <algorithm>
#include <boost/make_shared.hpp>


#define LOGGING_PREFIX "filter_features_magnitude"


namespace maptk
{

namespace core
{

//Helper struct for the filter function
struct feature_at_index_is_greater
{
  bool operator()(const std::pair<unsigned int, double> &l, const std::pair<unsigned int, double> &r)
  {
    return l.second > r.second;
  }
};

/// Private implementation class
class filter_features_magnitude::priv
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

  feature_set_sptr
  filter(feature_set_sptr feat, std::vector<unsigned int> &ind) const
  {
    const std::vector<feature_sptr> &feat_vec = feat->features();
    if (feat_vec.size() <= min_features)
    {
      return feat;
    }

    //  Create a new vector with the index and magnitude for faster sorting
    std::vector<std::pair<unsigned int, double> > indices;
    indices.reserve(feat_vec.size());
    for (unsigned int i = 0; i < feat_vec.size(); i++)
    {
      indices.push_back(std::make_pair<unsigned int, double>(i, feat_vec[i]->magnitude()));
    }

    // sorting descending on feature magnitude
    feature_at_index_is_greater comp;
    std::sort(indices.begin(), indices.end(), comp);

    // compute threshold
    unsigned int cutoff = std::max(min_features, static_cast<unsigned int>(top_percent * indices.size()));

    std::vector<feature_sptr> filtered(cutoff);
    ind.resize(cutoff);
    for (unsigned int i = 0; i < cutoff; i++)
    {
      unsigned int index = indices[i].first;
      ind[i] = index;
      filtered[i] = feat_vec[index];
    }

    LOG_INFO(LOGGING_PREFIX,
             "Reduced " << feat_vec.size() << " features to " << filtered.size() << " features.");

    return boost::make_shared<maptk::simple_feature_set>(maptk::simple_feature_set(filtered));
  }

  double top_percent;
  unsigned int min_features;
};


/// Constructor
filter_features_magnitude
::filter_features_magnitude()
: d_(new priv)
{
}


/// Copy Constructor
filter_features_magnitude
::filter_features_magnitude(const filter_features_magnitude& other)
: d_(new priv(*other.d_))
{
}


/// Destructor
filter_features_magnitude
::~filter_features_magnitude()
{
}


/// Get this algorithm's \link maptk::config_block configuration block \endlink
config_block_sptr
filter_features_magnitude
::get_configuration() const
{
  // get base config from base class
  config_block_sptr config =
      maptk::algo::filter_features::get_configuration();

  config->set_value("top_percent", d_->top_percent,
                    "Percent of strongest keypoints to keep, range (0.0, 1.0]");

  config->set_value("min_features", d_->min_features,
                    "minimum number of features to keep");

  return config;
}


/// Set this algorithm's properties via a config block
void
filter_features_magnitude
::set_configuration(config_block_sptr config)
{
  d_->top_percent = config->get_value<double>("top_percent", d_->top_percent);
  d_->min_features = config->get_value<unsigned int>("min_features", d_->min_features);
}


/// Check that the algorithm's configuration config_block is valid
bool
filter_features_magnitude
::check_configuration(config_block_sptr config) const
{
  return true;
}


/// Filter feature set
feature_set_sptr
filter_features_magnitude
::filter(feature_set_sptr feat, std::vector<unsigned int> &indices) const
{
  return d_->filter(feat, indices);
}

} // end namespace core

} // end namespace maptk
