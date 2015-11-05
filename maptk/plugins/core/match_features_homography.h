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
 * \brief core match_features_homography algorithm impl interface
 */

#ifndef MAPTK_PLUGINS_CORE_MATCH_FEATURES_HOMOGRAPHY_H_
#define MAPTK_PLUGINS_CORE_MATCH_FEATURES_HOMOGRAPHY_H_

#include <vital/algo/filter_features.h>

#include <vital/algo/estimate_homography.h>
#include <vital/algo/match_features.h>
#include <vital/config/config_block.h>

#include <maptk/plugins/core/plugin_core_config.h>

#include <memory>

namespace kwiver {
namespace maptk {

namespace core
{

/// Combines a feature matchers, homography estimation, and filtering
/**
 *  This is a meta-algorithm for feature matching that combines one or more
 *  other feature matchers with homography estimation and feature filtering.
 *  The algorithm applies another configurable feature matcher algorithm and
 *  then applies a homography estimation algorithm to the resulting matches.
 *  Outliers to the fit homography are discarded from the set of matches.
 *
 *  If a second matcher algorithm is provided, this algorithm will warp the
 *  feature locations by the estimated homography before applying the second
 *  matching algorithm to the aligned points.  This approach is useful for
 *  finding weak matches that were missed by the first matcher but are
 *  easier to detect once approximate location is known.  A good choice for
 *  the second matcher is vxl::match_features_constrained.
 *
 *  If a filter_features algorithm is provided, this will be run on the
 *  input features \b before running the first matcher.  The second matcher
 *  will then run on the \b original unfilter features.  This allows, for
 *  example, a slower but more robust feature matcher to run on a subset
 *  of the strongest feature points in order to quickly establish an
 *  and estimated homography.  Then a second, fast matcher can pick up
 *  the additional weak matches using the constraint that the location
 *  in the image is now known approximately.
 */
class PLUGIN_CORE_EXPORT match_features_homography
  : public vital::algorithm_impl<match_features_homography, vital::algo::match_features>
{
public:
  /// Default Constructor
  match_features_homography();

  /// Copy Constructor
  match_features_homography(const match_features_homography&);

  /// Destructor
  virtual ~match_features_homography();

  /// Return the name of this implementation
  virtual std::string impl_name() const { return "homography_guided"; }

  /// Get this alg's \link vital::config_block configuration block \endlink
  virtual vital::config_block_sptr get_configuration() const;
  /// Set this algo's properties via a config block
  virtual void set_configuration(vital::config_block_sptr config);
  /// Check that the algorithm's currently configuration is valid
  virtual bool check_configuration(vital::config_block_sptr config) const;

  /// Match one set of features and corresponding descriptors to another
  /**
   * \param [in] feat1 the first set of features to match
   * \param [in] desc1 the descriptors corresponding to \a feat1
   * \param [in] feat2 the second set of features to match
   * \param [in] desc2 the descriptors corresponding to \a feat2
   * \returns a set of matching indices from \a feat1 to \a feat2
   */
  virtual vital::match_set_sptr
  match(vital::feature_set_sptr feat1, vital::descriptor_set_sptr desc1,
        vital::feature_set_sptr feat2, vital::descriptor_set_sptr desc2) const;

  /// Set the feature matching algorithms to use
  void set_first_feature_matcher(vital::algo::match_features_sptr alg)
  {
    matcher1_ = alg;
  }

  /// Set the optional second pass feature matching algorithm to use
  void set_second_feature_matcher(vital::algo::match_features_sptr alg)
  {
    matcher2_ = alg;
  }

  /// Set the optional feature filter to use
  void set_feature_filter(vital::algo::filter_features_sptr alg)
  {
    feature_filter_ = alg;
  }

  /// Set the homography estimation algorithm to use
  void set_homography_estimator(vital::algo::estimate_homography_sptr alg)
  {
    h_estimator_ = alg;
  }

private:
  /// private implementation class
  class priv;
  const std::unique_ptr<priv> d_;

  /// The feature matching algorithms to use
  vital::algo::match_features_sptr matcher1_, matcher2_;

  /// The homography estimation algorithm to use
  vital::algo::estimate_homography_sptr h_estimator_;

  /// The feature set filter algorithm to use
  vital::algo::filter_features_sptr feature_filter_;
};


} // end namespace algo

} // end namespace maptk
} // end namespace kwiver


#endif // MAPTK_PLUGINS_CORE_MATCH_FEATURES_HOMOGRAPHY_H_
