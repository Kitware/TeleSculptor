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
 * \brief config_block IO exceptions interface
 */

#ifndef MAPTK_CORE_EXCEPTIONS_CONFIG_IO_H
#define MAPTK_CORE_EXCEPTIONS_CONFIG_IO_H

#include "base.h"
#include <maptk/core/types.h>

namespace maptk
{

/// Base config_io exception class
class MAPTK_CORE_EXPORT config_block_io_exception
  : public maptk_core_base_exception
{
  public:
    ///Constructor
    /**
     * \param file_path The path to the file related to this error.
     * \param reason    Reason for the exception.
     */
    config_block_io_exception(path_t const& file_path, char const* reason) MAPTK_NOTHROW;
    /// Deconstructor
    virtual ~config_block_io_exception() MAPTK_NOTHROW;

    /// Path to file this exception revolves around.
    path_t m_file_path;
    /// Reason for exception
    std::string m_reason;
};

/// Exception for when a file could not be found
class MAPTK_CORE_EXPORT file_not_found_exception
  : public config_block_io_exception
{
  public:
    /// Constructor
    /**
     * \param file_path The file path that was looked for.
     * \param reason    The reason the file wasn't found.
     */
    file_not_found_exception(path_t const& file_path, char const* reason) MAPTK_NOTHROW;
    /// Deconstructor
    virtual ~file_not_found_exception() MAPTK_NOTHROW;
};

/// Exception for when a file could not be read for whatever reason.
class MAPTK_CORE_EXPORT file_not_read_exception
  : public config_block_io_exception
{
  public:
    ///Constructor
    /**
     * \param file_path The file path on which the read was attempted.
     * \param reason    The reason for the read exception.
     */
    file_not_read_exception(path_t const& file_path, char const* reason) MAPTK_NOTHROW;
    /// Deconstructor
    virtual ~file_not_read_exception() MAPTK_NOTHROW;
};

/// Exception for when a file could not be parsed after being read in
class MAPTK_CORE_EXPORT file_not_parsed_exception
  : public config_block_io_exception
{
  public:
    /// Constructor
    /**
     * \param file_path The file path to which the parsing exception occurred.
     * \param reason    The reason for the parsing exception.
     */
    file_not_parsed_exception(path_t const& file_path, char const* reason) MAPTK_NOTHROW;
    /// Deconstructor
    virtual ~file_not_parsed_exception() MAPTK_NOTHROW;
};

/// Exception for when a file was not able to be written
class MAPTK_CORE_EXPORT file_write_exception
  : public config_block_io_exception
{
  public:
    /// Constructor
    /**
     * \param file_path The file path to which the write was attempted.
     * \param reason    The reason for the write exception
     */
    file_write_exception(path_t const& file_path, char const* reason) MAPTK_NOTHROW;
    /// Deconstructor
    virtual ~file_write_exception() MAPTK_NOTHROW;
};

}

#endif // MAPTK_CORE_EXCEPTIONS_CONFIG_IO_H
