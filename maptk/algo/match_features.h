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
 * \brief match_features algorithm definition interface
 */

#ifndef MAPTK_ALGO_MATCH_FEATURES_H_
#define MAPTK_ALGO_MATCH_FEATURES_H_

#include <maptk/config.h>

#include <vital/algorithm.h>
#include <vital/feature_set.h>
#include <vital/descriptor_set.h>
#include <vital/match_set.h>
#include <boost/shared_ptr.hpp>

namespace maptk
{

namespace algo
{

/// An abstract base class for matching feature points
class MAPTK_LIB_EXPORT match_features
  : public kwiver::vital::algorithm_def<match_features>
{
public:
  /// Return the name of this algorithm
  static std::string static_type_name() { return "match_features"; }

  /// Match one set of features and corresponding descriptors to another
  /**
   * \param feat1 the first set of features to match
   * \param desc1 the descriptors corresponding to \a feat1
   * \param feat2 the second set fof features to match
   * \param desc2 the descriptors corresponding to \a feat2
   * \returns a set of matching indices from \a feat1 to \a feat2
   */
  virtual kwiver::vital::match_set_sptr
  match(kwiver::vital::feature_set_sptr feat1, kwiver::vital::descriptor_set_sptr desc1,
        kwiver::vital::feature_set_sptr feat2, kwiver::vital::descriptor_set_sptr desc2) const = 0;

};


/// Shared pointer type for match_features algorithm definition class
typedef boost::shared_ptr<match_features> match_features_sptr;


} // end namespace algo

} // end namespace maptk


#endif // MAPTK_ALGO_MATCH_FEATURES_H_
