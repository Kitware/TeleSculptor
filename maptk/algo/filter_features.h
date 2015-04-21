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

#ifndef MAPTK_ALGO_FILTER_FEATURES_H_
#define MAPTK_ALGO_FILTER_FEATURES_H_

#include <boost/shared_ptr.hpp>
#include <utility>
#include <vector>

#include <maptk/algo/algorithm.h>
#include <maptk/feature_set.h>
#include <maptk/descriptor_set.h>

/**
 * \file
 * \brief Header defining abstract \link maptk::algo::filter_features
 *       \endlink algorithm
 */

namespace maptk
{

namespace algo
{

/// \brief Abstract base class for feature set filter algorithms.
class MAPTK_LIB_EXPORT filter_features
  : public algorithm_def<filter_features>
{
public:

  /// Return the name of this algorithm.
  virtual std::string type_name() const { return "filter_features"; }

  /// filter a feature set
  /**
   * \param [in] feature set to filter
   * \returns a filtered version of the feature set (simple_feature_set)
   */
  virtual feature_set_sptr
  filter( feature_set_sptr input ) const;

  /// filter a feature_set and its coresponding descriptor_set
  /**
   * \param [in] feature set to filter
   * \param [in] parallel descriptor set to filter
   * returns a pair of the filtered features and descriptors
   */
  virtual std::pair<feature_set_sptr, descriptor_set_sptr>
  filter( feature_set_sptr feat, descriptor_set_sptr descr) const;

protected:

  virtual feature_set_sptr
  filter(feature_set_sptr feat, std::vector<unsigned int> &indices) const = 0;

};

typedef boost::shared_ptr<filter_features> filter_features_sptr;

} // end namespace algo

} // end namespace maptk


#endif // MAPTK_ALGO_FILTER_FEATURES_H_
