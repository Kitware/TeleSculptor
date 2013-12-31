/*ckwg +5
 * Copyright 2011-2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include "config_block_io.h"

#include <sstream>

namespace maptk
{

config_block_io_exception
::config_block_io_exception(char const* reason) MAPTK_NOTHROW
  : maptk_core_base_exception()
  , m_reason(reason)
{
}

config_block_io_exception
::~config_block_io_exception() MAPTK_NOTHROW
{
}

file_not_found_exception
::file_not_found_exception(path_t const& file_path, char const* reason) MAPTK_NOTHROW
  : config_block_io_exception(reason)
  , m_file_path(file_path)
{
  std::ostringstream sstr;
  sstr << "Could not file file at location \'" << m_file_path.c_str() << "\': "
          << m_reason;
  m_what = sstr.str();
}

file_not_found_exception
::~file_not_found_exception() MAPTK_NOTHROW
{
}

file_not_read_exception
::file_not_read_exception(path_t const& file_path, char const* reason) MAPTK_NOTHROW
  : config_block_io_exception(reason)
  , m_file_path(file_path)
{
  std::ostringstream sstr;
  sstr << "Failed to read from file \'" << m_file_path.c_str() << "\': "
       << m_reason;
  m_what = sstr.str();
}

file_not_read_exception
::~file_not_read_exception() MAPTK_NOTHROW
{
}

file_write_exception
::file_write_exception(path_t const& file_path, char const* reason) MAPTK_NOTHROW
  : config_block_io_exception(reason)
  , m_file_path(file_path)
{
  std::ostringstream sstr;
  sstr << "Failed to write to file \'" << m_file_path.c_str() << "\': "
       << m_reason;
  m_what = sstr.str();
}

file_write_exception
::~file_write_exception() MAPTK_NOTHROW
{
}

}
