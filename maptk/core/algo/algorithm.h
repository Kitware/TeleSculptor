/*ckwg +5
 * Copyright 2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_ALGO_ALGORITHM_H_
#define MAPTK_ALGO_ALGORITHM_H_

#include <string>
#include <boost/shared_ptr.hpp>

namespace maptk
{

namespace algo
{

/// An abstract base class for all algorithms
class algorithm
{
public:
  /// Return the name of this algorithm
  virtual std::string type_name() const = 0;

  /// Return the name of this implementation
  virtual std::string impl_name() const = 0;

};


/// An intermediate templated algorithm base class
/*
 *  Uses the curiously recurring template pattern for cloning
 */
template <typename T>
class algorithm_of : public algorithm
{
public:
  /// Returns a clone of this algorithm
  virtual boost::shared_ptr<T> clone() const = 0;
};

} // end namespace algo

} // end namespace maptk


#endif // MAPTK_ALGO_ALGORITHM_H_
