/*ckwg +29
 * Copyright 2014 by Kitware, Inc.
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

#ifndef MAPTK_PLUGINS_VISCL_MATCH_SET_H_
#define MAPTK_PLUGINS_VISCL_MATCH_SET_H_

#include <maptk/match_set.h>
#include <maptk/plugins/viscl/viscl_config.h>

#include <viscl/core/buffer.h>


namespace maptk
{

namespace vcl
{


/// A concrete match set that wraps VisCL matches
class MAPTK_VISCL_EXPORT match_set
: public maptk::match_set
{
public:
  /// Default constructor
  match_set() {}

  /// Constructor from VisCL matches
  explicit match_set(const viscl::buffer& viscl_matches)
   : data_(viscl_matches) {}

  /// Return the number of matches in the set
  /**
    * Warning: this function is slow, it downloads all of the matches
    * to count them it is recommended to use matches() if you need both
    * the size and the matches.
    */
  virtual size_t size() const;

  /// Return a vector of matching indices
  virtual std::vector<match> matches() const;

  /// Return the underlying VisCL match data
  const viscl::buffer& viscl_matches() const { return data_; }

private:
  // The collection of VisCL match data
  viscl::buffer data_;
};


/// Convert any match set to VisCL match data
/**
  * Will remove duplicate matches to a kpt from 2nd set
  */
MAPTK_VISCL_EXPORT viscl::buffer
matches_to_viscl(const maptk::match_set& match_set);


} // end namespace vcl

} // end namespace maptk


#endif // MAPTK_PLUGINS_VISCL_MATCH_SET_H_
