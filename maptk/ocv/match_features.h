/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_OCV_MATCH_FEATURES_H_
#define MAPTK_OCV_MATCH_FEATURES_H_

#include "ocv_config.h"
#include <maptk/core/algo/match_features.h>

#include <boost/scoped_ptr.hpp>

namespace maptk
{

namespace ocv
{

/// An abstract base class for matching feature points
class MAPTK_OCV_EXPORT match_features
  : public algo::algorithm_impl<match_features, algo::match_features>
{
public:
  /// Constructor
  match_features();

  /// Destructor
  virtual ~match_features();

  /// Copy Constructor
  match_features(const match_features& other);

  /// Return the name of this implementation
  std::string impl_name() const { return "ocv"; }

  /// Get this algorithm's \link maptk::config_block configuration block \endlink
  virtual config_block_sptr get_configuration() const;
  /// Set this algorithm's properties via a config block
  virtual void set_configuration(config_block_sptr config);
  /// Check that the algorithm's configuration config_block is valid
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

private:
  /// private implementation class
  class priv;
  boost::scoped_ptr<priv> d_;
};

} // end namespace ocv

} // end namespace maptk


#endif // MAPTK_OCV_MATCH_FEATURES_H_
