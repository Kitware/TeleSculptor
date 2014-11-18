/*ckwg +29
 * Copyright 2013-2014 by Kitware, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither name of Kitware, Inc. nor the names of any contributors may be used
 *    to endorse or promote products derived from this software without specific
 *    prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHORS OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
