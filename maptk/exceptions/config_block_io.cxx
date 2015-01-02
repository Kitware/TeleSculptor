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
 * \brief config_block IO exceptions implementation
 */

#include "config_block_io.h"

#include <sstream>

namespace maptk
{

config_block_io_exception
::config_block_io_exception(path_t const& file_path, char const* reason) MAPTK_NOTHROW
  : maptk_core_base_exception()
  , m_file_path(file_path)
  , m_reason(reason)
{
}

config_block_io_exception
::~config_block_io_exception() MAPTK_NOTHROW
{
}

file_not_found_exception
::file_not_found_exception(path_t const& file_path, char const* reason) MAPTK_NOTHROW
  : config_block_io_exception(file_path, reason)
{
  std::ostringstream sstr;
  sstr << "Could not find file at location \'" << m_file_path.c_str() << "\': "
          << m_reason;
  m_what = sstr.str();
}

file_not_found_exception
::~file_not_found_exception() MAPTK_NOTHROW
{
}

file_not_read_exception
::file_not_read_exception(path_t const& file_path, char const* reason) MAPTK_NOTHROW
  : config_block_io_exception(file_path, reason)
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

file_not_parsed_exception
::file_not_parsed_exception(path_t const& file_path, char const* reason) MAPTK_NOTHROW
  : config_block_io_exception(file_path, reason)
{
  std::ostringstream sstr;
  sstr << "Failed to parse file \'" << m_file_path.c_str() << "\': "
       << m_reason;
  m_what = sstr.str();
}

file_not_parsed_exception
::~file_not_parsed_exception() MAPTK_NOTHROW
{
}

file_write_exception
::file_write_exception(path_t const& file_path, char const* reason) MAPTK_NOTHROW
  : config_block_io_exception(file_path, reason)
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
