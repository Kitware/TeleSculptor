/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_ALGO_MATCH_FEATURES_HOMOGRAPHY_H_
#define MAPTK_ALGO_MATCH_FEATURES_HOMOGRAPHY_H_

#include <maptk/core/config_block.h>
#include <maptk/core/algo/match_features.h>
#include <maptk/core/algo/estimate_homography.h>

namespace maptk
{

namespace algo
{

/// Combines a feature matcher and homography estimation for constrained matching
class match_features_homography
: public algo::algorithm_impl<match_features_homography, match_features>
{
public:
  /// Default Constructor
  match_features_homography() {}

  /// Copy Constructor
  match_features_homography(const match_features_homography&) {}

  /// Return the name of this implementation
  std::string impl_name() const { return "homography_guided"; }

  /// Get this alg's \link maptk::config_block configuration block \endlink
  virtual config_block_sptr get_configuration() const;
  /// Set this algo's properties via a config block
  virtual void set_configuration(config_block_sptr config);
  /// Check that the algorithm's currently configuration is valid
  virtual void check_configuration(config_block_sptr config) const;

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
  void set_feature_matcher(match_features_sptr alg)
  {
    matcher_ = alg;
  }

  /// Set the homography estimation algorithm to use
  void set_homography_estimator(estimate_homography_sptr alg)
  {
    h_estimator_ = alg;
  }

private:
  /// The feature matching algorithm to use
  match_features_sptr matcher_;

  /// The homography estimation algorithm to use
  estimate_homography_sptr h_estimator_;
};


} // end namespace algo

} // end namespace maptk


#endif // MAPTK_ALGO_MATCH_FEATURES_HOMOGRAPHY_H_
