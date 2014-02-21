/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief base algorithm/_def/_impl class interfaces
 */

#ifndef MAPTK_ALGO_ALGORITHM_H_
#define MAPTK_ALGO_ALGORITHM_H_

#include <maptk/core/core_config.h>

#include <string>
#include <vector>

#include <boost/shared_ptr.hpp>

#include <maptk/core/config_block.h>

namespace maptk
{

namespace algo
{

/// An abstract base class for all algorithms
class MAPTK_CORE_EXPORT algorithm
{
public:
  /// Return the name of this algorithm
  virtual std::string type_name() const = 0;

  /// Return the name of this implementation
  virtual std::string impl_name() const = 0;

  /// Get this algorithm's \link maptk::config_block configuration block \endlink
  /**
   * This base virtual function implementation returns an empty configuration
   * block whose name is set to \c this->type_name.
   *
   * \returns \c config_block containing the configuration for this algorithm
   *          and any nested components.
   */
  virtual config_block_sptr get_configuration() const;

  /// Set this algorithm's properties via a config block
  /**
   * \throws no_such_configuration_value_exception
   *    Thrown if an expected configuration value is not present.
   * \throws algorithm_configuration_exception
   *    Thrown when the algorithm is given an invalid \c config_block or is'
   *    otherwise unable to configure itself.
   *
   * \param config  The \c config_block instance containing the configuration
   *                parameters for this algorithm
   */
  virtual void set_configuration(config_block_sptr config) = 0;

  /// Check that the algorithm's configuration config_block is valid
  /**
   * This checks solely within the provided \c config_block and not against
   * the current state of the instance. This isn't static for inheritence
   * reasons.
   *
   * \param config  The config block to check configuration of.
   *
   * \returns true if the configuration check passed and false if it didn't.
   */
  virtual bool check_configuration(config_block_sptr config) const = 0;

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
class MAPTK_CORE_EXPORT algorithm_def
  : public algorithm
{
public:
  /// Shared pointer type of the templated maptk::algorithm_def class
  typedef boost::shared_ptr<Self> base_sptr;

  /// Returns a clone of this algorithm
  virtual base_sptr clone() const = 0;

  /// Register instances of this algorithm
  static bool register_instance(base_sptr inst);

  /// Factory method to make an instance of this algorithm by impl_name
  static base_sptr create(const std::string& impl_name);

  /// Return a vector of the impl_name of each registered implementation
  static std::vector<std::string> registered_names();

  /// Check the given name against registered implementation names
  /**
   * If the given name is not a valid implementation name.
   *
   * \param impl_name implementation name to check for the validity of.
   * \returns true if the given \c impl_name is valid for this definition, or
   *          false if it is not a valid implementation name.
   */
  static bool has_impl_name(std::string const& impl_name);

  /// Return a \c config_block for all registered implementations
  /**
   * For each registered implementation of this definition, add a subblock,
   * labeled under the name of that implementation, of that implementation's
   * config_block.
   *
   * If no implementation of this definition has any configuraiton properties,
   * the \c config_block returned will be empty.
   */
  static config_block_sptr get_impl_configurations();

  /// Helper function for properly getting a nested algorithm's configuration
  /**
   * Adds a configurable algorithm implementation switch for this algorithm_def.
   * If the variable pointed to by \c nested_algo is a defined sptr to an
   * implementation, its \link maptk::config_block configuration \endlink
   * parameters are merged with the given
   * \link maptk::config_block config_block \endlink.
   *
   * \param     name        An identifying name for the nested algorithm
   * \param[in] config      The \c config_block instance in which to put the
   *                          nested algorithm's configuration.
   * \param[in] nested_algo The nested algorithm's sptr variable.
   */
  static void get_nested_algo_configuration(std::string const& name,
                                            config_block_sptr config,
                                            base_sptr nested_algo);

  /// Helper function for properly setting a nested algorithm's configuration
  /**
   * The nested algorithm will not be set if the implementation switch (as
   * defined in the \c get_nested_algo_configuration) is not present or set to
   * an invalid value relative to the registered names for this
   * \c algorithm_def.
   *
   * \param name                An identifying name for the nested algorithm.
   * \param[in] config          The \c config_block instance from which we will
   *                              draw configuration needed for the nested
   *                              algorithm instance.
   * \param[in,out] nested_algo The nested algorithm's sptr variable.
   */
  static void set_nested_algo_configuration(std::string const& name,
                                            config_block_sptr config,
                                            base_sptr &nested_algo);

  /// Helper macro for checking that basic nested algorithm configuration is valid
  /**
   * Check that the expected implementation switch exists and that its value is
   * registered implementation name.
   *
   * If the name is valid, we also recursively call check_configuration() on the
   * set implementation. This is done with a fresh create so we don't have to
   * rely on the implementation being defined in the instance this is called
   * from.
   *
   * \param     name        An identifying name for the nested algorithm.
   * \param[in] config  The \c config_block to check.
   */
  static bool check_nested_algo_configuration(std::string const& name,
                                              config_block_sptr config);

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
class algorithm_impl
  : public Base
{
public:
  /// shared pointer type of this impl's base maptk::algorithm_def class.
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
