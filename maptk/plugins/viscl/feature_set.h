/*ckwg +29
 * Copyright 2014-2015 by Kitware, Inc.
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

#ifndef MAPTK_PLUGINS_VISCL_FEATURE_SET_H_
#define MAPTK_PLUGINS_VISCL_FEATURE_SET_H_

#include <vital/types/feature_set.h>
#include <maptk/plugins/viscl/viscl_config.h>

#include <viscl/core/buffer.h>
#include <viscl/core/image.h>


namespace kwiver {
namespace maptk {

namespace vcl
{

/// A concrete feature set that wraps VisCL features
/**
  * A VisCL feature only has the location set
  * It is possible to get the smoothing scale but that value is not
  * saved on the GPU so would have to be provided externally
  */
class MAPTK_VISCL_EXPORT feature_set
: public vital::feature_set
{
public:

  struct type
  {
    viscl::buffer features_;
    viscl::buffer numfeat_;
    viscl::image kptmap_;
  };

  /// Default Constructor
  feature_set() {}

  /// Constructor from VisCL data
  explicit feature_set(const type& viscl_features)
  : data_(viscl_features) {}

  /// Return the number of features in the set
  /**
    * Downloads the size from the GPU
    */
  virtual size_t size() const;

  /// Return a vector of feature shared pointers
  virtual std::vector<vital::feature_sptr> features() const;

  /// Return the underlying VisCL features data structure
  const type& viscl_features() const { return data_; }

protected:

  /// The VisCL feature point data
  type data_;
};

/// Convert any feature set to a VisCL data (upload if needed)
/**
  * viscl only cares about integer feature location, therefore will lose
  * info converting from maptk feature set to viscl and back
  */
MAPTK_VISCL_EXPORT feature_set::type
features_to_viscl(const vital::feature_set& features);


} // end namespace vcl

} // end namespace maptk
} // end namespace kwiver


#endif // MAPTK_PLUGINS_VISCL_FEATURE_SET_H_
