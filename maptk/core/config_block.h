/*ckwg +5
 * Copyright 2011-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief Header for \link maptk::config_block configuration \endlink object
 */

#ifndef MAPTK_CORE_CONFIG_H
#define MAPTK_CORE_CONFIG_H

#include "core_config.h"

#include <cstddef>
#include <exception>
#include <map>
#include <set>
#include <string>
#include <typeinfo>
#include <vector>

#include <boost/optional/optional.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>

#include "types.h"
#include "exceptions/config_block.h"


namespace maptk
{

class config_block;
/// Shared pointer for the \c config_block class
typedef boost::shared_ptr<config_block> config_block_sptr;

/// Configuration value storage structure
/**
 * The associated shared pointer for this object is \c config_block_sptr
 */
class MAPTK_CORE_EXPORT config_block
  : public boost::enable_shared_from_this<config_block>,
           boost::noncopyable
{
  public:

    /// Create an empty configuration.
    /**
     * \param name The name of the configuration block.
     * \returns An empty configuration block.
     */
    static config_block_sptr empty_config(config_block_key_t const& name = config_block_key_t());

    /// Destructor
    virtual ~config_block();

    /// Get the name of this \c config_block instance.
    config_block_key_t get_name();

    /// Get a subblock from the configuration.
    /**
     * Retrieve an unlinked configuration subblock from the current
     * configuration. Changes made to it do not affect \c *this.
     *
     * \param key The name of the sub-configuration to retrieve.
     * \returns A subblock with copies of the values.
     */
    config_block_sptr subblock(config_block_key_t const& key) const;

    /// Get a subblock view into the configuration.
    /**
     * Retrieve a view into the current configuration. Changes made to \c *this
     * are seen through the view and vice versa.
     *
     * \param key The name of the sub-configuration to retrieve.
     * \returns A subblock which links to the \c *this.
     */
    config_block_sptr subblock_view(config_block_key_t const& key);

    /// Internally cast the value.
    /**
     * \throws no_such_configuration_value_exception Thrown if the requested index does not exist.
     * \throws bad_configuration_cast_exception Thrown if the cast fails.
     *
     * \param key The index of the configuration value to retrieve.
     * \returns The value stored within the configuration.
     */
    template <typename T>
    T get_value(config_block_key_t const& key) const;

    /// Cast the value, returning a default value in case of an error.
    /**
     * \throws no_such_configuration_value_exception Thrown if the requested index does not exist.
     * \throws bad_configuration_cast_exception Thrown if the cast fails.
     *
     * \param key The index of the configuration value to retrieve.
     * \param def The value \p key does not exist or the cast fails.
     * \returns The value stored within the configuration, or \p def if something goes wrong.
     */
    template <typename T>
    T get_value(config_block_key_t const& key, T const& def) const MAPTK_NOTHROW;

    /// Get the description associated to a value
    /**
     * If the provided key has no description associated with it, an empty
     * \c config_block_description_t value is returned.
     *
     * \throws no_such_configuration_value_exception Thrown if the requested
     *                                               key does not exist.
     *
     * \param key The name of the parameter to get the description of.
     * \returns The description of the requested key.
     */
    config_block_description_t get_description(config_block_key_t const& key) const;

    /// Set a value within the configuration.
    /**
     * If this key already exists, has a description and no new description
     * was passed with this \c set_value call, the previous description is
     * retained. We assume that the previous description is still valid and
     * this a value overwrite. If it is intended for the description to also
     * be overwritted, an \c unset_value call should be performed on the key
     * first, and then this \c set_value call.
     *
     * \throws set_on_read_only_value_exception Thrown if \p key is marked as read-only.
     *
     * \postconds
     * \postcond{<code>this->get_value<value_t>(key) == value</code>}
     * \endpostconds
     *
     * \param key The index of the configuration value to set.
     * \param value The value to set for the \p key.
     * \param descr Description of the key. If this is set, we will override
     *              any existing description for the given key. If a
     *              description for the given key already exists and nothing
     *              was provided for this parameter, the existing description
     *              is maintained.
     */
    template <typename T>
    void set_value(config_block_key_t const& key,
                   T const& value,
                   config_block_description_t const& descr = config_block_key_t());

    /// Remove a value from the configuration.
    /**
     * \throws unset_on_read_only_value_exception Thrown if \p key is marked as read-only.
     * \throws no_such_configuration_value_exception Thrown if the requested index does not exist.
     *
     * \postconds
     * \postcond{<code>this->get_value<T>(key)</code> throws \c no_such_configuration_value_exception}
     * \endpostconds
     *
     * \param key The index of the configuration value to unset.
     */
    void unset_value(config_block_key_t const& key);

    /// Query if a value is read-only.
    /**
     *
     * \param key The key of the value query.
     * \returns True if \p key is read-only, false otherwise.
     */
    bool is_read_only(config_block_key_t const& key) const;

    /// Set the value within the configuration as read-only.
    /**
     * \postconds
     * \postcond{<code>this->is_read_only(key) == true</code>}
     * \endpostconds
     *
     * \param key The key of the value to mark as read-only.
     */
    void mark_read_only(config_block_key_t const& key);

    /// Merge the values in \p config into the current config.
    /**
     * \note Any values currently set within \c *this will be overwritten if conficts occur.
     *
     * \throws set_on_read_only_value_exception Thrown if \p key is marked as read-only.
     *
     * \postconds
     * \postcond{\c this->available_values() ⊆ \c config->available_values()}
     * \endpostconds
     *
     * \param config The other configuration.
     */
    void merge_config(config_block_sptr const& config);

    ///Return the values available in the configuration.
    /**
     * \returns All of the keys available within the block.
     */
    config_block_keys_t available_values() const;

    /// Check if a value exists for \p key.
    /**
     * \param key The index of the configuration value to check.
     * \returns Whether the key exists.
     */
    bool has_value(config_block_key_t const& key) const;

    /// The separator between blocks.
    static config_block_key_t const block_sep;
    /// The magic group for global parameters.
    static config_block_key_t const global_value;
  private:
    /// Internal constructor
    MAPTK_NO_EXPORT config_block(config_block_key_t const& name, config_block_sptr parent);

    /// Private helper method to extract a value for a key
    /**
     * \param key key to find the associated value to.
     * \returns boost::none if the key doesn't exist or the key's value.
     */
    boost::optional<config_block_value_t> find_value(config_block_key_t const& key) const;
    /// private value getter function
    /**
     * \param key key to get the associated value to.
     * \returns key's value or an empty config_block_value_t if the key is not found.
     */
    MAPTK_NO_EXPORT config_block_value_t m_get_value(config_block_key_t const& key) const;
    /// private key/value setter
    /**
     * \param key key to set a value to
     * \param value the value as a config_block_value_t
     * \param descr optional description of the key.
     */
    void m_set_value(config_block_key_t const& key,
                     config_block_value_t const& value,
                     config_block_description_t const& descr = config_block_key_t());

    typedef std::map<config_block_key_t, config_block_value_t> store_t;
    typedef std::set<config_block_key_t> ro_list_t;

    config_block_sptr m_parent;
    config_block_key_t m_name;
    store_t m_store;
    store_t m_descr_store;
    ro_list_t m_ro_list;
};

/// Default cast handling of configuration values.
/**
 * \note Do not use this in user code. Use \ref config_block_cast instead.
 * \param value The value to convert.
 * \returns The value of \p value in the requested type.
 */
template <typename R, typename T>
inline
R
config_block_cast_default(T const& value)
{
  try
  {
    return boost::lexical_cast<R>(value);
  }
  catch (boost::bad_lexical_cast const& e)
  {
    throw bad_config_block_cast(e.what());
  }
}

/// Cast a configuration value to the requested type.
/**
 * \throws bad_configuration_cast Thrown when the conversion fails.
 * \param value The value to convert.
 * \returns The value of \p value in the requested type.
 */
template <typename R, typename T>
inline
R
config_block_cast(T const& value)
{
  return config_block_cast_default<R, T>(value);
}

/// Type-specific casting handling, cb_value_t->bool specialization
/**
 * This is the \c bool to \c config_block_value_t specialization to handle
 * \tt{true}, \tt{false}, \tt{yes} and \tt{no} literal conversion versus just
 * \tt{1} and \tt{0} (1 and 0 still handled if provided).
 *
 * \note Do not use this in user code. Use \ref config_block_cast instead.
 * \param value The value to convert.
 * \returns The value of \p value in the requested type.
 */
template<>
MAPTK_CORE_EXPORT
bool config_block_cast(config_block_value_t const& value);

/// Type-specific casting handling, bool->cb_value_t specialization
/**
 * This is the \c config_block_value_t to \c bool specialization that outputs
 * \tt{true} and \tt{false} literals instead of 1 or 0.
 *
 * \note Do not use this in user code. Use \ref config_block_cast instead.
 * \param value The value to convert.
 * \returns The value of \p value as either "true" or "false".
 */
template<>
inline
config_block_value_t config_block_cast(bool const& value)
{
  return value ? "true" : "false";
}

/// Internally cast the value.
template <typename T>
T
config_block
::get_value(config_block_key_t const& key) const
{
  boost::optional<config_block_value_t> value = find_value(key);

  if (!value)
  {
    throw no_such_configuration_value_exception(key);
  }

  try
  {
    return config_block_cast<T, config_block_value_t>(*value);
  }
  catch (bad_config_block_cast const& e)
  {
    throw bad_config_block_cast_exception(key, *value, typeid(T).name(), e.what());
  }
}

/// Cast the value, returning a default value in case of an error.
template <typename T>
T
config_block
::get_value(config_block_key_t const& key, T const& def) const MAPTK_NOTHROW
{
  try
  {
    return get_value<T>(key);
  }
  catch (...)
  {
    return def;
  }
}

/// Set a value within the configuration.
template <typename T>
void
config_block
::set_value(config_block_key_t const& key,
            T const& value,
            config_block_description_t const& descr)
{
  this->m_set_value(key, config_block_cast<config_block_value_t, T>(value), descr);
}

}

#endif // MAPTK_CORE_CONFIG_
