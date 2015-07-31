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

#ifndef MAPTK_PLUGINS_VISCL_DESCRIPTOR_SET_H_
#define MAPTK_PLUGINS_VISCL_DESCRIPTOR_SET_H_


#include <vital/types/descriptor_set.h>
#include <maptk/plugins/viscl/viscl_config.h>

#include <viscl/core/buffer.h>


namespace kwiver {
namespace maptk {

namespace vcl
{

/// A concrete descriptor set that wraps VisCL descriptors.
class MAPTK_VISCL_EXPORT descriptor_set
: public kwiver::vital::descriptor_set
{
public:

  /// Default Constructor
  descriptor_set() {}

  /// Constructor from VisCL descriptors
  explicit descriptor_set(const viscl::buffer& viscl_descriptors)
  : data_(viscl_descriptors) {}

  /// Return the number of descriptor in the set
  virtual size_t size() const { return data_.len(); }

  /// Return a vector of descriptor shared pointers
  /**
    * Warning: These descriptors must be matched by hamming distance
    */
  virtual std::vector<kwiver::vital::descriptor_sptr> descriptors() const;

  /// Return the native VisCL descriptors structure
  const viscl::buffer& viscl_descriptors() const { return data_; }

protected:

  /// The handle to a VisCL set of descriptors
  viscl::buffer data_;
};


/// Convert a descriptor set to a VisCL descriptor set must be <int,4>
MAPTK_VISCL_EXPORT viscl::buffer
descriptors_to_viscl(const kwiver::vital::descriptor_set& desc_set);


} // end namespace vcl

} // end namespace maptk
} // end namespace kwiver


#endif // MAPTK_PLUGINS_VISCL_DESCRIPTOR_SET_H_
