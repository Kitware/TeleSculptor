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

config_key_t const config::block_sep = config_key_t(":");
config_key_t const config::global_value = config_key_t("_global");

static bool does_not_begin_with(config_key_t const& key, config_key_t const& name);
static config_key_t strip_block_name(config_key_t const& subblock, config_key_t const& key);

config_t
config
::empty_config(config_key_t const& name)
{
  return config_t(new config(name, config_t()));
}

config
::~config()
{
}

config_t
config
::subblock(config_key_t const& key) const
{
  config_t conf = empty_config(key);

  BOOST_FOREACH (config_key_t const& key_name, available_values())
  {
    if (does_not_begin_with(key_name, key))
    {
      continue;
    }

    config_key_t const stripped_key_name = strip_block_name(key, key_name);

    conf->set_value(stripped_key_name, get_value(key_name));
  }

  return conf;
}

config_t
config
::subblock_view(config_key_t const& key)
{
  return config_t(new config(key, shared_from_this()));
}

void
config
::set_value(config_key_t const& key, config_value_t const& value)
{
  if (m_parent)
  {
    m_parent->set_value(m_name + block_sep + key, value);
  }
  else
  {
    if (is_read_only(key))
    {
      config_value_t const current_value = get_value<config_value_t>(key, config_value_t());

      throw set_on_read_only_value_exception(key, current_value, value);
    }

    m_store[key] = value;
  }
}

void
config
::unset_value(config_key_t const& key)
{
  if (m_parent)
  {
    m_parent->unset_value(m_name + block_sep + key);
  }
  else
  {
    if (is_read_only(key))
    {
      config_value_t const current_value = get_value<config_value_t>(key, config_value_t());

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
::is_read_only(config_key_t const& key) const
{
  return (0 != m_ro_list.count(key));
}

void
config
::mark_read_only(config_key_t const& key)
{
  m_ro_list.insert(key);
}

void
config
::merge_config(config_t const& conf)
{
  config_keys_t const keys = conf->available_values();

  BOOST_FOREACH (config_key_t const& key, keys)
  {
    config_value_t const& val = conf->get_value<config_value_t>(key);

    set_value(key, val);
  }
}

config_keys_t
config
::available_values() const
{
  config_keys_t keys;

  if (m_parent)
  {
    config_keys_t parent_keys = m_parent->available_values();

    config_keys_t::iterator const i = std::remove_if(parent_keys.begin(), parent_keys.end(), boost::bind(does_not_begin_with, _1, m_name));

    parent_keys.erase(i, parent_keys.end());

    std::transform(parent_keys.begin(), parent_keys.end(), std::back_inserter(keys), boost::bind(strip_block_name, m_name, _1));
  }
  else
  {
    BOOST_FOREACH (store_t::value_type const& value, m_store)
    {
      config_key_t const& key = value.first;

      keys.push_back(key);
    }
  }

  return keys;
}

bool
config
::has_value(config_key_t const& key) const
{
  if (m_parent)
  {
    return m_parent->has_value(m_name + block_sep + key);
  }

  return (0 != m_store.count(key));
}

config
::config(config_key_t const& name, config_t parent)
  : m_parent(parent)
  , m_name(name)
  , m_store()
  , m_ro_list()
{
}

boost::optional<config_value_t>
config
::find_value(config_key_t const& key) const
{
  if (!has_value(key))
  {
    return boost::none;
  }

  return get_value(key);
}

config_value_t
config
::get_value(config_key_t const& key) const
{
  if (m_parent)
  {
    return m_parent->get_value(m_name + block_sep + key);
  }

  store_t::const_iterator i = m_store.find(key);

  if (i == m_store.end())
  {
    return config_value_t();
  }

  return i->second;
}

template <>
bool
config_cast_inner(config_value_t const& value)
{
  static config_value_t const true_string = config_value_t("true");
  static config_value_t const false_string = config_value_t("false");

  config_value_t const value_lower = boost::to_lower_copy(value);

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
does_not_begin_with(config_key_t const& key, config_key_t const& name)
{
  static config_key_t const global_start = config::global_value + config::block_sep;

  return (!boost::starts_with(key, name + config::block_sep) &&
          !boost::starts_with(key, global_start));
}

config_key_t
strip_block_name(config_key_t const& subblock, config_key_t const& key)
{
  if (!boost::starts_with(key, subblock + config::block_sep))
  {
    return key;
  }

  return key.substr(subblock.size() + config::block_sep.size());
}

}
