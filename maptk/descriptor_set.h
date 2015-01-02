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
 * \brief core descriptor_set interface
 */

#ifndef MAPTK_DESCRIPTOR_SET_H_
#define MAPTK_DESCRIPTOR_SET_H_


#include "descriptor.h"
#include <boost/shared_ptr.hpp>

namespace maptk
{

/// An abstract ordered collection of feature descriptors.
/**
 * The base class of descriptor_set is abstract and provides a
 * double precision interface.  The templated derived class
 * can store values in either single or double precision.
 */
class descriptor_set
{
public:
  /// Destructor
  virtual ~descriptor_set() {}

  /// Return the number of descriptors in the set
  virtual size_t size() const = 0;

  /// Return a vector of descriptor shared pointers
  virtual std::vector<descriptor_sptr> descriptors() const = 0;
};

/// Shared pointer for base descriptor_set type
typedef boost::shared_ptr<descriptor_set> descriptor_set_sptr;


/// A concrete descriptor set that simply wraps a vector of descriptors.
class simple_descriptor_set
  : public descriptor_set
{
public:
  /// Default Constructor
  simple_descriptor_set() {}

  /// Constructor from a vector of descriptors
  explicit simple_descriptor_set(const std::vector<descriptor_sptr>& descriptors)
  : data_(descriptors) {}

  /// Return the number of descriptor in the set
  virtual size_t size() const { return data_.size(); }

  /// Return a vector of descriptor shared pointers
  virtual std::vector<descriptor_sptr> descriptors() const { return data_; }

protected:

  /// The vector of featrues
  std::vector<descriptor_sptr> data_;
};


} // end namespace maptk


#endif // MAPTK_DESCRIPTOR_SET_H_
