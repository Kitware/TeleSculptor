/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief config_block related exceptions implementation
 */

#include "config_block.h"

#include <sstream>

namespace maptk
{

config_block_exception
::config_block_exception() MAPTK_NOTHROW
  : maptk_core_base_exception()
{
}

config_block_exception
::~config_block_exception() MAPTK_NOTHROW
{
}

bad_config_block_cast
::bad_config_block_cast(char const* reason) MAPTK_NOTHROW
  : config_block_exception()
{
  this->m_what = reason;
}

bad_config_block_cast
::~bad_config_block_cast() MAPTK_NOTHROW
{
}

bad_config_block_cast_exception
::bad_config_block_cast_exception(config_block_key_t const& key,
                                  config_block_value_t const& value,
                                  char const* type,
                                  char const* reason) MAPTK_NOTHROW
  : config_block_exception()
  , m_key(key)
  , m_value(value)
  , m_type(type)
  , m_reason(reason)
{
  std::ostringstream sstr;
  sstr << "Failed to cast \'" << m_key << "\' "
          "with value \'" << m_value << "\' as "
          "a \'" << m_type <<"\': " << m_reason << ".";
  m_what = sstr.str();
}

bad_config_block_cast_exception
::~bad_config_block_cast_exception() MAPTK_NOTHROW
{
}

no_such_configuration_value_exception
::no_such_configuration_value_exception(config_block_key_t const& key) MAPTK_NOTHROW
  : config_block_exception()
  , m_key(key)
{
  std::ostringstream sstr;
  sstr << "There is no configuration value for the key "
          "\'" << m_key << "\'.";
  m_what = sstr.str();
}

no_such_configuration_value_exception
::~no_such_configuration_value_exception() MAPTK_NOTHROW
{
}

set_on_read_only_value_exception
::set_on_read_only_value_exception(config_block_key_t const& key,
                                   config_block_value_t const& value,
                                   config_block_value_t const& new_value) MAPTK_NOTHROW
  : config_block_exception()
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
::unset_on_read_only_value_exception(config_block_key_t const& key,
                                     config_block_value_t const& value) MAPTK_NOTHROW
  : config_block_exception()
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

}
