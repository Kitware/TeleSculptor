/*ckwg +29
 * Copyright 2016 by Kitware, Inc.
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
 * \brief Header defining the core match_features_fundamental_matrix algorithm
 */

#ifndef MAPTK_PLUGINS_CORE_MATCH_FEATURES_FUNDMENTAL_MATRIX_H_
#define MAPTK_PLUGINS_CORE_MATCH_FEATURES_FUNDMENTAL_MATRIX_H_

#include <vital/vital_config.h>
#include <maptk/plugins/core/maptk_core_export.h>

#include <vital/algo/filter_features.h>

#include <vital/algo/estimate_fundamental_matrix.h>
#include <vital/algo/match_features.h>
#include <vital/config/config_block.h>

#include <memory>

namespace kwiver {
namespace maptk {

namespace core
{

/// Combines a feature matcher, fundamental matrix estimation, and filtering
/**
 *  This is a meta-algorithm for feature matching that combines one other feature
 *  matcher with fundamental matrix estimation and feature filtering.
 *  The algorithm applies another configurable feature matcher algorithm and
 *  then applies a fundamental matrix estimation algorithm to the resulting matches.
 *  Outliers to the fit fundamental matrix are discarded from the set of matches.
 *
 *  If a filter_features algorithm is provided, this will be run on the
 *  input features \b before running the matcher.
 */
class MAPTK_CORE_EXPORT match_features_fundamental_matrix
  : public vital::algorithm_impl<match_features_fundamental_matrix, vital::algo::match_features>
{
public:
  /// Default Constructor
  match_features_fundamental_matrix();

  /// Copy Constructor
  match_features_fundamental_matrix(const match_features_fundamental_matrix&);

  /// Destructor
  virtual ~match_features_fundamental_matrix();

  /// Return the name of this implementation
  virtual std::string impl_name() const { return "fundamental_matrix_guided"; }

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


private:
  /// private implementation class
  class priv;
  const std::unique_ptr<priv> d_;
};


} // end namespace algo

} // end namespace maptk
} // end namespace kwiver


#endif // MAPTK_PLUGINS_CORE_MATCH_FEATURES_FUNDMENTAL_MATRIX_H_
