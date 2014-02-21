/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief match_features algorithm definition interface
 */

#ifndef MAPTK_ALGO_MATCH_FEATURES_H_
#define MAPTK_ALGO_MATCH_FEATURES_H_

#include <maptk/core/algo/algorithm.h>
#include <maptk/core/feature_set.h>
#include <maptk/core/descriptor_set.h>
#include <maptk/core/match_set.h>
#include <boost/shared_ptr.hpp>

namespace maptk
{

namespace algo
{

/// An abstract base class for matching feature points
class MAPTK_CORE_EXPORT match_features
  : public algorithm_def<match_features>
{
public:
  /// Return the name of this algorithm
  std::string type_name() const { return "match_features"; }

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
        feature_set_sptr feat2, descriptor_set_sptr desc2) const = 0;

};


typedef boost::shared_ptr<match_features> match_features_sptr;


} // end namespace algo

} // end namespace maptk


#endif // MAPTK_ALGO_MATCH_FEATURES_H_
