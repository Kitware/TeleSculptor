/*ckwg +29
 * Copyright 2014 by Kitware, Inc.
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
 * \brief Implementation for IO exceptions
 */

#include "io.h"
#include <sstream>

namespace maptk
{


io_exception
::io_exception() MAPTK_NOTHROW
{
  m_what = "An IO exception occurred.";
}

io_exception
::~io_exception() MAPTK_NOTHROW
{
}


path_not_exists
::path_not_exists(path_t path) MAPTK_NOTHROW
  : m_path(path)
{
  std::ostringstream sstr;
  sstr << "Path does not exist: " << path;
  m_what = sstr.str();
}

path_not_exists
::~path_not_exists() MAPTK_NOTHROW
{
}


path_not_a_file
::path_not_a_file(path_t path) MAPTK_NOTHROW
  : m_path(path)
{
  m_what = "Path does not point to a file: " + path.string();
}

path_not_a_file
::~path_not_a_file() MAPTK_NOTHROW
{
}


path_not_a_directory
::path_not_a_directory(path_t path) MAPTK_NOTHROW
  : m_path(path)
{
  m_what = "Path does not point to a directory: " + path.string();
}

path_not_a_directory
::~path_not_a_directory() MAPTK_NOTHROW
{
}


invalid_file
::invalid_file(path_t path, std::string reason) MAPTK_NOTHROW
  : m_path(path)
  , m_reason(reason)
{
  std::ostringstream ss;
  ss << "Invalid file " << m_path << ": " << m_reason;
  m_what = ss.str();
}

invalid_file
::~invalid_file() MAPTK_NOTHROW
{
}


invalid_data
::invalid_data(std::string reason) MAPTK_NOTHROW
  : m_reason(reason)
{
  m_what = "Invalid data: " + reason;
}

invalid_data
::~invalid_data() MAPTK_NOTHROW
{
}


} // end maptk namespace
