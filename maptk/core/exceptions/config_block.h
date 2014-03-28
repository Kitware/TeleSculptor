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
 * \brief config_block related exceptions interface
 */

#ifndef MAPTK_CORE_EXCEPTIONS_CONFIG_H
#define MAPTK_CORE_EXCEPTIONS_CONFIG_H

#include "base.h"

#include <maptk/core/types.h>

namespace maptk
{

/**
 * \brief The base class for all exceptions thrown from \ref config_block
 * \ingroup exceptions
 */
class MAPTK_CORE_EXPORT config_block_exception
  : public maptk_core_base_exception
{
  public:
    /// Constructor.
    config_block_exception() MAPTK_NOTHROW;
    /// Destructor.
    virtual ~config_block_exception() MAPTK_NOTHROW;
};

/**
 * \brief The inner exception thrown when casting fails.
 * \ingroup exceptions
 */
class MAPTK_CORE_EXPORT bad_config_block_cast
  : public config_block_exception
{
  public:
    /**
     * \brief Constructor.
     * \param reason The reason for the bad cast.
     */
    bad_config_block_cast(char const* reason) MAPTK_NOTHROW;
    /// Destructor.
    virtual ~bad_config_block_cast() MAPTK_NOTHROW;
};

/**
 * \brief Thrown when a value cannot be converted to the requested type.
 * \ingroup exceptions
 */
class MAPTK_CORE_EXPORT bad_config_block_cast_exception
  : public config_block_exception
{
  public:
    /**
     * \brief Constructor.
     *
     * \param key The key that was requested.
     * \param value The value that was failed to cast.
     * \param type The type that was requested.
     * \param reason The reason for the bad cast.
     */
    bad_config_block_cast_exception(config_block_key_t const& key,
                                    config_block_value_t const& value,
                                    char const* type,
                                    char const* reason) MAPTK_NOTHROW;
    /// Destructor.
    virtual ~bad_config_block_cast_exception() MAPTK_NOTHROW;

    /// The requested key name.
    config_block_key_t const m_key;
    /// The value of the requested key.
    config_block_value_t const m_value;
    /// The type requested for the cast.
    std::string const m_type;
    /// The reason for the failed cast.
    std::string const m_reason;
};

/**
 * \brief Thrown when a value is requested for a value which does not exist.
 * \ingroup exceptions
 */
class MAPTK_CORE_EXPORT no_such_configuration_value_exception
  : public config_block_exception
{
  public:
    /**
     * \brief Constructor.
     * \param key The key that was requested from the configuration.
     */
    no_such_configuration_value_exception(config_block_key_t const& key) MAPTK_NOTHROW;
    /// Destructor.
    virtual ~no_such_configuration_value_exception() MAPTK_NOTHROW;

    /// The requested key name.
    config_block_key_t const m_key;
};

/**
 * \brief Thrown when a value is set but is marked as read-only.
 * \ingroup exceptions
 */
class MAPTK_CORE_EXPORT set_on_read_only_value_exception
  : public config_block_exception
{
  public:
    /**
     * \brief Constructor.
     *
     * \param key The key that was requested from the configuration.
     * \param value The current read-only value of \p key.
     * \param new_value The value that was attempted to be set.
     */
    set_on_read_only_value_exception(config_block_key_t const& key,
                                     config_block_value_t const& value,
                                     config_block_value_t const& new_value) MAPTK_NOTHROW;
    /**
     * \brief Destructor.
     */
    virtual ~set_on_read_only_value_exception() MAPTK_NOTHROW;

    /// The requested key name.
    config_block_key_t const m_key;
    /// The existing value.
    config_block_value_t const m_value;
    /// The new value.
    config_block_value_t const m_new_value;
};

/**
 * \brief Thrown when a value is unset but is marked as read-only.
 * \ingroup exceptions
 */
class MAPTK_CORE_EXPORT unset_on_read_only_value_exception
  : public config_block_exception
{
  public:
    /**
     * \brief Constructor.
     *
     * \param key The key that was requested from the configuration.
     * \param value The current value for \p key.
     */
    unset_on_read_only_value_exception(config_block_key_t const& key,
                                       config_block_value_t const& value) MAPTK_NOTHROW;
    /**
     * \brief Destructor.
     */
    virtual ~unset_on_read_only_value_exception() MAPTK_NOTHROW;

    /// The requested key name.
    config_block_key_t const m_key;
    /// The existing value.
    config_block_value_t const m_value;
};

}

#endif // MAPTK_CORE_EXCEPTIONS_CONFIG_H
