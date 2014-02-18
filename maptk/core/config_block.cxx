/*ckwg +5
 * Copyright 2011-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include "config_block.h"

#include <boost/algorithm/string/case_conv.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include <boost/none.hpp>

#include <algorithm>
#include <iterator>
#include <sstream>

/**
 * \file maptk/core/config_block.cxx
 *
 * \brief Implementation of \link maptk::config_block configuration \endlink object
 */

namespace maptk
{

config_block_key_t const config_block::block_sep = config_block_key_t(":");
config_block_key_t const config_block::global_value = config_block_key_t("_global");

/// private helper method for determining key path prefixes
static bool does_not_begin_with(config_block_key_t const& key,
                                config_block_key_t const& name);
/// private helper method to strip a block name from a key path
static config_block_key_t strip_block_name(config_block_key_t const& subblock,
                                           config_block_key_t const& key);

/// Create an empty configuration.
config_block_sptr
config_block
::empty_config(config_block_key_t const& name)
{
  // remember, config_block_sptr is a boost shared pointer
  return config_block_sptr(new config_block(name, config_block_sptr()));
}

/// Destructor
config_block
::~config_block()
{
}

/// Get the name of this \c config_block instance.
config_block_key_t
config_block
::get_name()
{
  return this->m_name;
}

/// Get a subblock from the configuration.
config_block_sptr
config_block
::subblock(config_block_key_t const& key) const
{
  config_block_sptr conf = empty_config(key);

  BOOST_FOREACH (config_block_key_t const& key_name, available_values())
  {
    if (does_not_begin_with(key_name, key))
    {
      continue;
    }

    config_block_key_t const stripped_key_name = strip_block_name(key, key_name);

    conf->set_value(stripped_key_name,
                    m_get_value(key_name),
                    get_description(key_name));
  }

  return conf;
}

/// Get a subblock view into the configuration.
config_block_sptr
config_block
::subblock_view(config_block_key_t const& key)
{
  return config_block_sptr(new config_block(key, shared_from_this()));
}

config_block_description_t
config_block
::get_description(config_block_key_t const& key) const
{
  if (m_parent)
  {
    return m_parent->get_description(m_name + block_sep + key);
  }

  store_t::const_iterator i = m_descr_store.find(key);
  if (i == m_descr_store.end())
  {
    throw no_such_configuration_value_exception(key);
  }

  return i->second;
}

/// Remove a value from the configuration.
void
config_block
::unset_value(config_block_key_t const& key)
{
  if (m_parent)
  {
    m_parent->unset_value(m_name + block_sep + key);
  }
  else
  {
    if (is_read_only(key))
    {
      config_block_value_t const current_value = get_value<config_block_value_t>(key, config_block_value_t());

      throw unset_on_read_only_value_exception(key, current_value);
    }

    store_t::iterator const i = m_store.find(key);
    store_t::iterator const j = m_descr_store.find(key);

    // value and descr stores managed in parallel, so if key doesn't exist in
    // value store, there will be no parallel value in the descr store.
    if (i == m_store.end())
    {
      throw no_such_configuration_value_exception(key);
    }

    m_store.erase(i);
    m_descr_store.erase(j);
  }
}

/// Query if a value is read-only.
bool
config_block
::is_read_only(config_block_key_t const& key) const
{
  return (0 != m_ro_list.count(key));
}

/// Set the value within the configuration as read-only.
void
config_block
::mark_read_only(config_block_key_t const& key)
{
  m_ro_list.insert(key);
}

/// Merge the values in \p config_block into the current config.
void
config_block
::merge_config(config_block_sptr const& conf)
{
  config_block_keys_t const keys = conf->available_values();

  BOOST_FOREACH (config_block_key_t const& key, keys)
  {
    config_block_value_t const& val = conf->get_value<config_block_value_t>(key);
    config_block_description_t const& descr = conf->get_description(key);

    m_set_value(key, val, descr);
  }
}

///Return the values available in the configuration.
config_block_keys_t
config_block
::available_values() const
{
  config_block_keys_t keys;

  if (m_parent)
  {
    config_block_keys_t parent_keys = m_parent->available_values();

    config_block_keys_t::iterator const i = std::remove_if(parent_keys.begin(), parent_keys.end(), boost::bind(does_not_begin_with, _1, m_name));

    parent_keys.erase(i, parent_keys.end());

    std::transform(parent_keys.begin(), parent_keys.end(), std::back_inserter(keys), boost::bind(strip_block_name, m_name, _1));
  }
  else
  {
    BOOST_FOREACH (store_t::value_type const& value, m_store)
    {
      config_block_key_t const& key = value.first;

      keys.push_back(key);
    }
  }

  return keys;
}

/// Check if a value exists for \p key.
bool
config_block
::has_value(config_block_key_t const& key) const
{
  if (m_parent)
  {
    return m_parent->has_value(m_name + block_sep + key);
  }

  return (0 != m_store.count(key));
}

/// Internal constructor
config_block
::config_block(config_block_key_t const& name, config_block_sptr parent)
  : m_parent(parent)
  , m_name(name)
  , m_store()
  , m_descr_store()
  , m_ro_list()
{
}

/// private helper method to extract a value for a key
boost::optional<config_block_value_t>
config_block
::find_value(config_block_key_t const& key) const
{
  if (!has_value(key))
  {
    return boost::none;
  }

  return m_get_value(key);
}

config_block_value_t
config_block
::m_get_value(config_block_key_t const& key) const
{
  if (m_parent)
  {
    return m_parent->m_get_value(m_name + block_sep + key);
  }

  store_t::const_iterator i = m_store.find(key);

  if (i == m_store.end())
  {
    return config_block_value_t();
  }

  return i->second;
}

/// Set a value within the configuration.
void
config_block
::m_set_value(config_block_key_t const& key,
              config_block_value_t const& value,
              config_block_description_t const& descr)
{
  if (m_parent)
  {
    m_parent->set_value(m_name + block_sep + key, value, descr);
  }
  else
  {
    if (is_read_only(key))
    {
      config_block_value_t const current_value = get_value<config_block_value_t>(key, config_block_value_t());

      throw set_on_read_only_value_exception(key, current_value, value);
    }

    m_store[key] = value;

    // Only assign the description given if there is no stored description
    // for this key, or the given description is non-zero.
    if (m_descr_store.count(key) == 0 || descr.size() > 0)
    {
      m_descr_store[key] = descr;
    }
  }
}


/// Type-specific casting handling, bool specialization
template <>
bool
config_block_cast_inner(config_block_value_t const& value)
{
  static config_block_value_t const true_string = config_block_value_t("true");
  static config_block_value_t const false_string = config_block_value_t("false");
  static config_block_value_t const yes_string = config_block_value_t("yes");
  static config_block_value_t const no_string = config_block_value_t("no");

  config_block_value_t const value_lower = boost::to_lower_copy(value);

  if (value_lower == true_string || value_lower == yes_string)
  {
    return true;
  }
  else if (value_lower == false_string || value_lower == no_string)
  {
    return false;
  }

  return config_block_cast_default<bool, config_block_value_t>(value);
}

/// private helper method for determining key path prefixes
/**
 * \param key   The key string to check.
 * \param name  The prefix string to check for. Should not include a trailing
 *              block separator.
 * \returns True if the given key does not begin with the given name and is
 *          not a global variable.
 */
bool
does_not_begin_with(config_block_key_t const& key, config_block_key_t const& name)
{
  static config_block_key_t const global_start = config_block::global_value + config_block::block_sep;

  return (!boost::starts_with(key, name + config_block::block_sep) &&
          !boost::starts_with(key, global_start));
}

/// private helper method to strip a block name from a key path
/**
 * Conditionally strip the given subblock name from the given key path. If the
 * given key doesn't start with the given subblock, the given key is returned
 * as is.
 *
 * \param subblock  The subblock string to strip if present.
 * \param key       The key to conditionally strip from.
 * \returns The stripped key name.
 */
config_block_key_t
strip_block_name(config_block_key_t const& subblock, config_block_key_t const& key)
{
  if (!boost::starts_with(key, subblock + config_block::block_sep))
  {
    return key;
  }

  return key.substr(subblock.size() + config_block::block_sep.size());
}

}
