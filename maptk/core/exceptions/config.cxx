/*ckwg +5
 * Copyright 2011-2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include "config.h"

#include <sstream>

namespace maptk
{

configuration_exception
::configuration_exception() MAPTK_NOTHROW
  : maptk_core_base_exception()
{
}

configuration_exception
::~configuration_exception() MAPTK_NOTHROW
{
}

bad_configuration_cast
::bad_configuration_cast(char const* reason) MAPTK_NOTHROW
  : configuration_exception()
{
  this->m_what = reason;
}

bad_configuration_cast
::~bad_configuration_cast() MAPTK_NOTHROW
{
}

bad_configuration_cast_exception
::bad_configuration_cast_exception(config_key_t const& key,
                                   config_value_t const& value,
                                   char const* type,
                                   char const* reason) MAPTK_NOTHROW
  : configuration_exception()
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

bad_configuration_cast_exception
::~bad_configuration_cast_exception() MAPTK_NOTHROW
{
}

no_such_configuration_value_exception
::no_such_configuration_value_exception(config_key_t const& key) MAPTK_NOTHROW
  : configuration_exception()
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
::set_on_read_only_value_exception(config_key_t const& key,
                                   config_value_t const& value,
                                   config_value_t const& new_value) MAPTK_NOTHROW
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
::unset_on_read_only_value_exception(config_key_t const& key,
                                     config_value_t const& value) MAPTK_NOTHROW
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

}
