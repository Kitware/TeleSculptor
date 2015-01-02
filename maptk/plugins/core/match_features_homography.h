/*ckwg +29
 * Copyright 2013-2014 by Kitware, Inc.
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

#include <boost/scoped_ptr.hpp>

#include <maptk/algo/estimate_homography.h>
#include <maptk/algo/match_features.h>
#include <maptk/config_block.h>

#include <maptk/plugins/core/plugin_core_config.h>


namespace maptk
{

namespace core
{

/// Combines a feature matcher and homography estimation for constrained matching
class PLUGIN_CORE_EXPORT match_features_homography
  : public algo::algorithm_impl<match_features_homography, algo::match_features>
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

  /// Get this alg's \link maptk::config_block configuration block \endlink
  virtual config_block_sptr get_configuration() const;
  /// Set this algo's properties via a config block
  virtual void set_configuration(config_block_sptr config);
  /// Check that the algorithm's currently configuration is valid
  virtual bool check_configuration(config_block_sptr config) const;

  /// Match one set of features and corresponding descriptors to another
  /**
   * \param [in] feat1 the first set of features to match
   * \param [in] desc1 the descriptors corresponding to \a feat1
   * \param [in] feat2 the second set fof features to match
   * \param [in] desc2 the descriptors corresponding to \a feat2
   * \returns a set of matching indices from \a feat1 to \a feat2
   */
  virtual match_set_sptr
  match(feature_set_sptr feat1, descriptor_set_sptr desc1,
        feature_set_sptr feat2, descriptor_set_sptr desc2) const;

  /// Set the feature matching algorithm to use
  void set_feature_matcher(algo::match_features_sptr alg)
  {
    matcher_ = alg;
  }

  /// Set the homography estimation algorithm to use
  void set_homography_estimator(algo::estimate_homography_sptr alg)
  {
    h_estimator_ = alg;
  }

private:
  /// The feature matching algorithm to use
  algo::match_features_sptr matcher_;

  /// The homography estimation algorithm to use
  algo::estimate_homography_sptr h_estimator_;

  /// private implementation class
  class priv;
  boost::scoped_ptr<priv> d_;
};


} // end namespace algo

} // end namespace maptk


#endif // MAPTK_PLUGINS_CORE_MATCH_FEATURES_HOMOGRAPHY_H_
