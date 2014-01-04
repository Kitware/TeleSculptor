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
  virtual config_block_sptr get_configuration() const;

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
  virtual void set_configuration(config_block_sptr config) = 0;

  /// Check that the algorithm's configuration config_block is valid
  /**
   * If the algorithm's configuration is valid, nothing occurrs.
   * This checks solely within the provided \c config_block and not against
   * the current state of the instance.
   *
   * \throws algorithm_configuration_exception
   *    When configuration of the algorithm is invalid.
   *
   * \param config  The config block to check configuration of.
   */
  virtual void check_configuration(config_block_sptr config) const = 0;

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

  /// Check the given name against registered implementation names
  /**
   * If the given name is not a valid implementation name.
   *
   * /returns true if the given \c impl_name is valid for this definition, or
   *          false if it is not a valid implementation name.
   */
  static bool check_impl_name(std::string const& impl_name);

  /// Return a \c config_block for all registered implementations
  /**
   * For each registered implementation of this definition, add a subblock,
   * labeled under the name of that implementation, of that implementation's
   * config_block.
   *
   * If no implementations have non-empty configurations, the \c config_block
   * returned will be empty.
   */
  static config_block_sptr get_impl_configurations();

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


/// Helper macro for properly getting a nested algorithm's configuration
/**
 * Adds a configurable algorithm implementation switch for the specified
 * algorithm_def. If the variable pointed to by \c var_name is a defined
 * sptr to an instnace, its \link maptk::config_block configuration \endlink
 * is merged with the given \c config_block.
 *
 * This should be called within an algorithm_def or algorithm_impl
 * get_configuration() method. This macro relies on the nested algorithm
 * instance being stored in a variable \c var_name.
 *
 * \param algo_def      The algorithm_def type of the nested algorithm
 * \param var_name      The name of the variable that the nested algorithm is
 *                        being stored in, ideally a class variable, but may
 *                        be a local variable. This should be the Boost sptr
 *                        to the class.
 * \param config_block  The \c config_block object in which to put the nested
 *                        algorithm's configuration. This will include an
 *                        algorithm_impl switch as well as the nested
 *                        configuration block if the \c var_name is defined
 *                        with a valid algorithm_impl instance (sptr).
 */
#define get_nested_algo_configuration(algo_def, var_name, config)       \
  do                                                                    \
  {                                                                     \
    if(var_name)                                                        \
    {                                                                   \
      config->set_value(#algo_def "_algorithm", var_name->impl_name()); \
      config->subblock_view(var_name->impl_name())                      \
            ->merge_config(var_name->get_configuration());              \
    }                                                                   \
    else                                                                \
    {                                                                   \
      config->set_value(#algo_def "_algorithm",                         \
          "# Pick one: [ "                                              \
          + boost::algorithm::join(algo_def::registered_names(), " | ") \
          + " ]");                                                      \
    }                                                                   \
  } while(false)


/// Helper macro for properly setting a nested algorithm's configuration
/**
 * The nested algorithm will not be set if the implementation switch (as
 * defined in the \c get_nested_algo_configuration) is not present or set to
 * an invalid value in relation to the registered names for the given
 * \c algo_def.
 *
 * \param algo_def  The \c algorithm_def of the nested algorithm
 * \param var_name  The variable to which we will store a created
 *                    \c algorithm_impl sptr.
 * \param config    The \c config_block from which we will draw configuration
 *                    needed for the nested algorithm instance.
 */
#define set_nested_algo_configuration(algo_def, var_name, config)                     \
  do                                                                                  \
  {                                                                                   \
    if(config->has_value(#algo_def "_algorithm"))                                     \
    {                                                                                 \
      std::string impl_name = config->get_value<std::string>(#algo_def "_algorithm"); \
      if(algo_def::check_impl_name(impl_name))                                        \
      {                                                                               \
        var_name = algo_def::create(impl_name);                                       \
        var_name->set_configuration(config->subblock_view(impl_name));                \
      }                                                                               \
    }                                                                                 \
  } while (false)


/// Helper macro for checking that basic nested algorithm configuration
/**
 * Check that the expected implementation switch exists and that its value is
 * registered implementation name.
 *
 * If the name is valid, we also recursively call check_configuration() on the
 * set implementation. This is done with a fresh create so we don't have to
 * rely on the implementation being defined in the instance this is called from.
 *
 * \preconds
 * \pre{File that uses this macro has <maptk/core/exceptions.h> or
 *      <maptk/core/exceptions/algorithm.h> included.}
 * \endpreconds
 *
 * \throws algorithm_configuration_exception
 *    When the algorithm implementation switch is not present or when the value
 *    given to the switch is not a valid, registered implementation name.
 *
 * \param algo_def  The \c algorithm_def of the nested algorithm.
 * \param config    The \c config_block to check.
 */
#define check_nested_algo_configuration(algo_def, config)                       \
  do                                                                            \
  {                                                                             \
    if(!config->has_value(#algo_def "_algorithm"))                              \
    {                                                                           \
      throw algorithm_configuration_exception(type_name(), impl_name(),         \
          "Missing implementation switch for nested algorithm \""               \
          #algo_def "\"");                                                      \
    }                                                                           \
    std::string iname = config->get_value<std::string>(#algo_def "_algorithm"); \
    if(!algo_def::check_impl_name(iname))                                       \
    {                                                                           \
      throw algorithm_configuration_exception(type_name(), impl_name(),         \
          "Invlid implementation name for nested algorithm \""                  \
          #algo_def "\". Given: \"" + iname + "\"");                            \
    }                                                                           \
    algo_def::create(iname)->check_configuration(config->subblock_view(iname)); \
  } while (false)


} // end namespace algo

} // end namespace maptk


#endif // MAPTK_ALGO_ALGORITHM_H_
