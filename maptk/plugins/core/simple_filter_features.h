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

#ifndef MAPTK_PLUGINS_CORE_SIMPLE_FILTER_FEATURES_H_
#define MAPTK_PLUGINS_CORE_SIMPLE_FILTER_FEATURES_H_


#include <boost/scoped_ptr.hpp>

#include <maptk/algo/filter_features.h>
#include <maptk/plugins/core/plugin_core_config.h>

/**
 * \file
 * \brief Header defining \link maptk::core::simple_filter_features
 *        \endlink algorithm
 */

namespace maptk
{

namespace core
{

/// \brief Algorithm that filters features based on strength
class PLUGIN_CORE_EXPORT simple_filter_features
  : public algo::algorithm_impl<simple_filter_features, algo::filter_features>
{
public:
  /// Constructor
  simple_filter_features();

  /// Destructor
  virtual ~simple_filter_features();

  /// Copy Constructor
  simple_filter_features(const simple_filter_features& other);

  /// Return the name of this implementation
  virtual std::string impl_name() const { return "core"; }

  /// Get this algorithm's \link maptk::config_block configuration block \endlink
  virtual config_block_sptr get_configuration() const;
  /// Set this algorithm's properties via a config block
  virtual void set_configuration(config_block_sptr config);
  /// Check that the algorithm's configuration config_block is valid
  virtual bool check_configuration(config_block_sptr config) const;

protected:

  /// filter a feature set
  /**
   * \param [in] feature set to filter
   * \param [out] indices of the kept features to the original feature set
   * \returns a filtered version of the feature set
   */
  virtual feature_set_sptr
  filter(feature_set_sptr input, std::vector<unsigned int> &indices) const;

private:
  /// private implementation class
  class priv;
  boost::scoped_ptr<priv> d_;
};

} // end namespace core

} // end namespace maptk


#endif // MAPTK_PLUGINS_CORE_SIMPLE_FILTER_FEATURES_H_
