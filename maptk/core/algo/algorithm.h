/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_ALGO_ALGORITHM_H_
#define MAPTK_ALGO_ALGORITHM_H_

#include <string>
#include <vector>

#include <maptk/core/config_block.h>

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

  /// Get this alg's \link maptk::config_block configuration block \endlink
  /**
   * This base virtial function implementation returns an empty configuration
   * block whose name is set to \c this->type_name.
   *
   * \returns \c config_block containing the configuration for this algorithm
   *          and any nested components.
   */
  virtual config_block_sptr get_configuration();

  /// Set this algo's properties via a config block
  /**
   * \throws no_such_configuration_value_exception
   *    Thrown if an expected configuration value is not present.
   * \thrown algorithm_configuration_exception
   *    Thrown when the algorithm is given an invalid \c config_block or is'
   *    otherwise unable to configure itself.
   *
   * \param config  The \c config_block instance containing the configuration
   *                parameters for this algorithm
   */
  virtual void configure(config_block_sptr config) {};//= 0;

};


/// An intermediate templated base class for algorithm definition
/**
 *  Uses the curiously recurring template pattern to declare the
 *  clone function and automatically provide functions to register
 *  algorithm, and create new instance by name.
 *  Each algorithm definition should be declared as shown below
 *  \code
    class my_algo_def
    : public algorithm_def<my_algo_def>
    {
      ...
    };
    \endcode
 *  \sa algorithm_impl
 */
template <typename Self>
class algorithm_def : public algorithm
{
public:
  typedef boost::shared_ptr<Self> base_sptr;

  /// Returns a clone of this algorithm
  virtual base_sptr clone() const = 0;

  /// Register instances of this algorithm
  static bool register_instance(base_sptr inst);

  /// Factory method to make an instance of this algorithm by impl_name
  static base_sptr create(const std::string& impl_name);

  /// Return a vector of the impl_name of each registered implementation
  static std::vector<std::string> registered_names();

};


/// An intermediate templated base class for algorithm implementations
/**
 *  Uses a variation of the curiously recurring template pattern to
 *  implement the clone() and register_self() functions for the derived class.
 *  Each algorithm implementation should be declared as shown below
 *  \code
    class my_algo_impl
    : public algorithm_impl<my_algo_impl, my_algo_def>
    {
      ...
    };
    \endcode
 *  where \c my_algo_def is the abstract algorithm class being implemented.
 *  \sa algorithm_def
 */
template <typename Self, typename Base>
class algorithm_impl : public Base
{
public:
  typedef boost::shared_ptr<Base> base_sptr;

  /// Returns a clone of this algorithm
  virtual base_sptr clone() const
  {
    return base_sptr(new Self(static_cast<const Self&>(*this)));
  }

  /// Register this algorithm implementation
  static bool register_self()
  {
    return algorithm_def<Base>::register_instance(base_sptr(new Self));
  }

};


} // end namespace algo

} // end namespace maptk


#endif // MAPTK_ALGO_ALGORITHM_H_
