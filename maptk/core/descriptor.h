/*ckwg +5
 * Copyright 2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_DESCRIPTOR_H_
#define MAPTK_DESCRIPTOR_H_

#include <vector>
#include <iostream>

namespace maptk
{

typedef unsigned char byte;

/// A representation of a feature descriptor used in matching.
class descriptor
{
public:
  /// Default Constructor
  descriptor();

  virtual ~descriptor() {}

  /// The size of the descriptor in bytes
  virtual std::size_t size() const = 0;

  /// Return the descriptor as a vector of bytes
  /// This should always work, 
  /// even if the underlying type is not bytes
  virtual std::vector<byte> as_bytes() const = 0;

  /// Return the descriptor as a vector of doubles
  /// Return an empty vector if this makes no sense
  /// for the underlying type.
  virtual std::vector<double> as_double() const = 0;

};


template <typename T, unsigned N>
class descriptor_fixed
  : public descriptor
{
public:
  /// Default Constructor
  descriptor_fixed<T,N>() {}

  /// The size of the descriptor in bytes
  std::size_t size() const { return N * sizeof(T); }

  /// Return the descriptor as a vector of bytes
  std::vector<byte> as_bytes() const
  {
    std::vector<byte> vec_bytes;
    // TODO: implement this
    return vec_bytes;
  }

protected:
  T data_[N];
};

/// output stream operator for a feature
std::ostream&  operator<<(std::ostream& s, const descriptor& d);

/// input stream operator for a feature
std::istream&  operator>>(std::istream& s, descriptor& d);


} // end namespace maptk


#endif // MAPTK_DESCRIPTOR_H_
