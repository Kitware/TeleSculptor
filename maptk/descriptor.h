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
 * \brief core descriptor interface and template implementations
 */

#ifndef MAPTK_DESCRIPTOR_H_
#define MAPTK_DESCRIPTOR_H_

#include <maptk/config.h>

#include <iostream>
#include <vector>

#include <boost/shared_ptr.hpp>

namespace maptk
{

/// Convenience typedef for a byte
typedef unsigned char byte;

/// A representation of a feature descriptor used in matching.
class descriptor
{
public:
  /// Destructor
  virtual ~descriptor() {}

  /// The number of elements of the underlying type
  virtual std::size_t size() const = 0;

  /// The number of bytes used to represent the data
  virtual std::size_t num_bytes() const = 0;

  /// Return the descriptor as a vector of bytes
  /**
   * This should always work,
   * even if the underlying type is not bytes
   */
  virtual std::vector<byte> as_bytes() const = 0;

  /// Return the descriptor as a vector of doubles
  /**
   * Return an empty vector if this makes no sense
   * for the underlying type.
   */
  virtual std::vector<double> as_double() const = 0;

};

/// Shared pointer for base descriptor type
typedef boost::shared_ptr<descriptor> descriptor_sptr;


/// Abstract base class of a descriptor containing an array of type T
template <typename T>
class descriptor_array_of
  : public descriptor
{
public:

  /// The number of bytes used to represent the data
  std::size_t num_bytes() const { return this->size() * sizeof(T); }

  /// Return the descriptor as a vector of bytes
  std::vector<byte> as_bytes() const
  {
    const byte* byte_data = reinterpret_cast<const byte*>(this->raw_data());
    return std::vector<byte>(byte_data, byte_data+this->num_bytes());
  }

  /// Return the descriptor as a vector of doubles
  std::vector<double> as_double() const
  {
    const size_t length = this->size();
    std::vector<double> double_data(length);
    for(size_t i=0; i<length; ++i)
    {
      double_data[i] = static_cast<double>(this->raw_data()[i]);
    }
    return double_data;
  }

  /// Return an pointer to the raw data array
  virtual T* raw_data() = 0;

  /// Return an pointer to the raw data array
  virtual const T* raw_data() const = 0;

};


/// A representation of a descriptor of fixed type and size
template <typename T, unsigned N>
class descriptor_fixed
  : public descriptor_array_of<T>
{
public:
  /// Default Constructor
  descriptor_fixed<T,N>() {}

  /// The number of elements of the underlying type
  std::size_t size() const { return N; }

  /// Return an pointer to the raw data array
  T* raw_data() { return data_; }

  /// Return an pointer to the raw data array
  const T* raw_data() const { return data_; }

protected:
  /// data array
  T data_[N];
};


/// A representation of a descriptor of fixed type and variable size
template <typename T>
class descriptor_dynamic
  : public descriptor_array_of<T>
{
public:
  /// Constructor
  descriptor_dynamic<T>(size_t len)
  : data_(new T[len]),
    length_(len) {}

  /// Destructor
  ~descriptor_dynamic<T>() { delete [] data_; }

  /// The number of elements of the underlying type
  std::size_t size() const { return length_; }

  /// Return an pointer to the raw data array
  T* raw_data() { return data_; }

  /// Return an pointer to the raw data array
  const T* raw_data() const { return data_; }

protected:
  /// data array
  T* data_;
  /// length of data array
  size_t length_;
};


/// output stream operator for a feature
MAPTK_LIB_EXPORT std::ostream& operator<<(std::ostream& s, const descriptor& d);

/// input stream operator for a feature
MAPTK_LIB_EXPORT std::istream& operator>>(std::istream& s, descriptor& d);


} // end namespace maptk


#endif // MAPTK_DESCRIPTOR_H_
