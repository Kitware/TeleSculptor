/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
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
