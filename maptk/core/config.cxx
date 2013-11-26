/*ckwg +5
 * Copyright 2011-2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include "config.h"

#include <boost/algorithm/string/case_conv.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include <boost/none.hpp>

#include <algorithm>
#include <iterator>
#include <sstream>

/**
 * \file config.cxx
 *
 * \brief Implementation of \link sprokit::config configuration\endlink in the pipeline.
 */

namespace maptk
{

config::key_t const config::block_sep = key_t(":");
config::key_t const config::global_value = key_t("_global");

static bool does_not_begin_with(config::key_t const& key, config::key_t const& name);
static config::key_t strip_block_name(config::key_t const& subblock, config::key_t const& key);

config_t
config
::empty_config(key_t const& name)
{
  return config_t(new config(name, config_t()));
}

config
::~config()
{
}

config_t
config
::subblock(key_t const& key) const
{
  config_t conf = empty_config(key);

  BOOST_FOREACH (key_t const& key_name, available_values())
  {
    if (does_not_begin_with(key_name, key))
    {
      continue;
    }

    key_t const stripped_key_name = strip_block_name(key, key_name);

    conf->set_value(stripped_key_name, get_value(key_name));
  }

  return conf;
}

config_t
config
::subblock_view(key_t const& key)
{
  return config_t(new config(key, shared_from_this()));
}

void
config
::set_value(key_t const& key, value_t const& value)
{
  if (m_parent)
  {
    m_parent->set_value(m_name + block_sep + key, value);
  }
  else
  {
    if (is_read_only(key))
    {
      value_t const current_value = get_value<value_t>(key, value_t());

      throw set_on_read_only_value_exception(key, current_value, value);
    }

    m_store[key] = value;
  }
}

void
config
::unset_value(key_t const& key)
{
  if (m_parent)
  {
    m_parent->unset_value(m_name + block_sep + key);
  }
  else
  {
    if (is_read_only(key))
    {
      value_t const current_value = get_value<value_t>(key, value_t());

      throw unset_on_read_only_value_exception(key, current_value);
    }

    store_t::iterator const i = m_store.find(key);

    if (i == m_store.end())
    {
      throw no_such_configuration_value_exception(key);
    }

    m_store.erase(i);
  }
}

bool
config
::is_read_only(key_t const& key) const
{
  return (0 != m_ro_list.count(key));
}

void
config
::mark_read_only(key_t const& key)
{
  m_ro_list.insert(key);
}

void
config
::merge_config(config_t const& conf)
{
  config::keys_t const keys = conf->available_values();

  BOOST_FOREACH (key_t const& key, keys)
  {
    value_t const& val = conf->get_value<value_t>(key);

    set_value(key, val);
  }
}

config::keys_t
config
::available_values() const
{
  keys_t keys;

  if (m_parent)
  {
    keys_t parent_keys = m_parent->available_values();

    keys_t::iterator const i = std::remove_if(parent_keys.begin(), parent_keys.end(), boost::bind(does_not_begin_with, _1, m_name));

    parent_keys.erase(i, parent_keys.end());

    std::transform(parent_keys.begin(), parent_keys.end(), std::back_inserter(keys), boost::bind(strip_block_name, m_name, _1));
  }
  else
  {
    BOOST_FOREACH (store_t::value_type const& value, m_store)
    {
      key_t const& key = value.first;

      keys.push_back(key);
    }
  }

  return keys;
}

bool
config
::has_value(key_t const& key) const
{
  if (m_parent)
  {
    return m_parent->has_value(m_name + block_sep + key);
  }

  return (0 != m_store.count(key));
}

config
::config(key_t const& name, config_t parent)
  : m_parent(parent)
  , m_name(name)
  , m_store()
  , m_ro_list()
{
}

boost::optional<config::value_t>
config
::find_value(key_t const& key) const
{
  if (!has_value(key))
  {
    return boost::none;
  }

  return get_value(key);
}

config::value_t
config
::get_value(key_t const& key) const
{
  if (m_parent)
  {
    return m_parent->get_value(m_name + block_sep + key);
  }

  store_t::const_iterator i = m_store.find(key);

  if (i == m_store.end())
  {
    return value_t();
  }

  return i->second;
}

configuration_exception
::configuration_exception() MAPTK_NOTHROW
  : std::exception()
{
}

configuration_exception
::~configuration_exception() MAPTK_NOTHROW
{
}

char const*
configuration_exception
::what() const MAPTK_NOTHROW
{
  return this->m_what.c_str();
}

bad_configuration_cast
::bad_configuration_cast(char const* reason) MAPTK_NOTHROW
  : configuration_exception()
{
  m_what = reason;
}

bad_configuration_cast
::~bad_configuration_cast() MAPTK_NOTHROW
{
}

no_such_configuration_value_exception
::no_such_configuration_value_exception(config::key_t const& key) MAPTK_NOTHROW
  : configuration_exception()
  , m_key(key)
{
  std::ostringstream sstr;

  sstr << "There is no configuration value with the key "
          "\'" << m_key << "\'.";

  m_what = sstr.str();
}

no_such_configuration_value_exception
::~no_such_configuration_value_exception() MAPTK_NOTHROW
{
}

bad_configuration_cast_exception
::bad_configuration_cast_exception(config::key_t const& key, config::value_t const& value, char const* type, char const* reason) MAPTK_NOTHROW
  : configuration_exception()
  , m_key(key)
  , m_value(value)
  , m_type(type)
  , m_reason(reason)
{
  std::ostringstream sstr;

  sstr << "Failed to cast key \'" << m_key << "\' "
          "with value \'" << m_value << "\' as "
          "a \'" << m_type << "\': " << m_reason << ".";

  m_what = sstr.str();
}

bad_configuration_cast_exception
::~bad_configuration_cast_exception() MAPTK_NOTHROW
{
}

set_on_read_only_value_exception
::set_on_read_only_value_exception(config::key_t const& key, config::value_t const& value, config::value_t const& new_value) MAPTK_NOTHROW
  : configuration_exception()
  , m_key(key)
  , m_value(value)
  , m_new_value(new_value)
{
  std::ostringstream sstr;

  sstr << "The key \'" << m_key << "\' "
          "was marked as read-only with the value "
          "\'" << m_value << "\' was attempted to be "
          "set to \'" << m_new_value << "\'.";

  m_what = sstr.str();
}

set_on_read_only_value_exception
::~set_on_read_only_value_exception() MAPTK_NOTHROW
{
}

unset_on_read_only_value_exception
::unset_on_read_only_value_exception(config::key_t const& key, config::value_t const& value) MAPTK_NOTHROW
  : configuration_exception()
  , m_key(key)
  , m_value(value)
{
  std::ostringstream sstr;

  sstr << "The key \'" << m_key << "\' "
          "was marked as read-only with the value "
          "\'" << m_value << "\' was attempted to be "
          "unset.";

  m_what = sstr.str();
}

unset_on_read_only_value_exception
::~unset_on_read_only_value_exception() MAPTK_NOTHROW
{
}

template <>
bool
config_cast_inner(config::value_t const& value)
{
  static config::value_t const true_string = config::value_t("true");
  static config::value_t const false_string = config::value_t("false");

  config::value_t const value_lower = boost::to_lower_copy(value);

  if (value_lower == true_string)
  {
    return true;
  }
  else if (value_lower == false_string)
  {
    return false;
  }

  return config_cast_default<bool>(value);
}

bool
does_not_begin_with(config::key_t const& key, config::key_t const& name)
{
  static config::key_t const global_start = config::global_value + config::block_sep;

  return (!boost::starts_with(key, name + config::block_sep) &&
          !boost::starts_with(key, global_start));
}

config::key_t
strip_block_name(config::key_t const& subblock, config::key_t const& key)
{
  if (!boost::starts_with(key, subblock + config::block_sep))
  {
    return key;
  }

  return key.substr(subblock.size() + config::block_sep.size());
}

}
