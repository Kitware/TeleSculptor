/*ckwg +5
 * Copyright 2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_DESCRIPTOR_SET_H_
#define MAPTK_DESCRIPTOR_SET_H_


#include "descriptor.h"
#include <boost/shared_ptr.hpp>

namespace maptk
{

/// An abstract ordered collection of feature descriptors.
///
/// The base class of descriptor_set is abstract and provides a
/// double precision interface.  The templated derived class
/// can store values in either single or double precision.
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
