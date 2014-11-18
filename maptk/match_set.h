/*ckwg +29
 * Copyright 2013-2014 by Kitware, Inc.
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
 * \brief core match_set interface
 */

#ifndef MAPTK_MATCH_SET_H_
#define MAPTK_MATCH_SET_H_


#include <vector>
#include <boost/shared_ptr.hpp>

namespace maptk
{

/// Index pair indicating matching features between two arrays
typedef std::pair<unsigned, unsigned> match;

/// A collection of matching indices between one set of objects and another.
class match_set
{
public:
  /// Destructor
  virtual ~match_set() {}

  /// Return the number of matches in the set
  virtual size_t size() const = 0;

  /// Return a vector of matching indices
  virtual std::vector<match> matches() const = 0;
};

/// Shared pointer of base match_set type
typedef boost::shared_ptr<match_set> match_set_sptr;


/// A concrete match set that simply wraps a vector of matches.
class simple_match_set
: public match_set
{
public:
  /// Default Constructor
  simple_match_set() {}

  /// Constructor from a vector of matches
  explicit simple_match_set(const std::vector<match>& matches)
  : data_(matches) {}

  /// Return the number of matches in the set
  virtual size_t size() const { return data_.size(); }

  /// Return a vector of match shared pointers
  virtual std::vector<match> matches() const { return data_; }

protected:

  /// The vector of matches
  std::vector<match> data_;
};


} // end namespace maptk


#endif // MAPTK_MATCH_SET_H_
