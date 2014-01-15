/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_FEATURE_SET_H_
#define MAPTK_FEATURE_SET_H_

#include <vector>

#include <boost/shared_ptr.hpp>

#include "feature.h"

namespace maptk
{

/// An abstract ordered collection of 2D image feature points.
///
/// The base class of feature_set is abstract and provides a
/// double precision interface.  The templated derived class
/// can store values in either single or double precision.
class feature_set
{
public:
  /// Destructor
  virtual ~feature_set() {}

  /// Return the number of features in the set
  virtual size_t size() const = 0;

  /// Return a vector of feature shared pointers
  virtual std::vector<feature_sptr> features() const = 0;
};

typedef boost::shared_ptr<feature_set> feature_set_sptr;


/// A concrete feature set that simply wraps a vector of features.
class simple_feature_set
  : public feature_set
{
public:
  /// Default Constructor
  simple_feature_set() {}

  /// Constructor from a vector of features
  explicit simple_feature_set(const std::vector<feature_sptr>& features)
  : data_(features) {}

  /// Return the number of feature in the set
  virtual size_t size() const { return data_.size(); }

  /// Return a vector of feature shared pointers
  virtual std::vector<feature_sptr> features() const { return data_; }

protected:

  /// The vector of featrues
  std::vector<feature_sptr> data_;
};


} // end namespace maptk


#endif // MAPTK_FEATURE_SET_H_
