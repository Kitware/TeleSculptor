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
 * \brief MAPTK Exceptions pertaining to IO operations
 */

#ifndef MAPTK_EXCEPTIONS_IO_H_
#define MAPTK_EXCEPTIONS_IO_H_

#include "base.h"
#include <string>
#include <maptk/types.h>

namespace maptk
{


/// MAPTK Generic IO exception
class MAPTK_LIB_EXPORT io_exception
  : public maptk_core_base_exception
{
public:
  /// Constructor
  io_exception() MAPTK_NOTHROW;
  /// Destructor
  virtual ~io_exception() MAPTK_NOTHROW;
};


/// Exception for when a given path doesn't point to anything on the filesystem
class MAPTK_LIB_EXPORT path_not_exists
  : public io_exception
{
public:
  /// Constructor
  /**
   * \param path The path that doesn't point to an existing file or directory
   */
  path_not_exists(path_t path) MAPTK_NOTHROW;
  /// Destructor
  virtual ~path_not_exists() MAPTK_NOTHROW;

  /// Path that didn't exist.
  path_t m_path;
};


/// Exception for when a given path doesn't point to a file.
class MAPTK_LIB_EXPORT path_not_a_file
  : public io_exception
{
public:
  /// Constructor
  /**
   * \param path The path that doesn't point to a file.
   */
  path_not_a_file(path_t path) MAPTK_NOTHROW;
  /// Destructor
  virtual ~path_not_a_file() MAPTK_NOTHROW;

  /// Path to a location that isn't a file.
  path_t m_path;
};


/// Exception for when a given path doesn't point to a directory.
class MAPTK_LIB_EXPORT path_not_a_directory
  : public io_exception
{
public:
  /// Constructor
  /**
   * \param path The path that doesn't point to a directory.
   */
  path_not_a_directory(path_t path) MAPTK_NOTHROW;
  /// Destructor
  virtual ~path_not_a_directory() MAPTK_NOTHROW;

  /// Path to a location that isn't a directory.
  path_t m_path;
};


/// Exception for an encounter with an invalid file by some metric.
class MAPTK_LIB_EXPORT invalid_file
  : public io_exception
{
public:
  /// Constructor
  /*
   * \param file    The file that has been deemed invalid
   * \param reason  The reason for invalidity.
   */
  invalid_file(path_t file, std::string reason) MAPTK_NOTHROW;
  /// Destructor
  virtual ~invalid_file() MAPTK_NOTHROW;

  /// Path to the invalid file location
  path_t m_path;
  /// Reason the file is invalid
  std::string m_reason;
};


/// Exception for an encounter with invalid data by some metric
class MAPTK_LIB_EXPORT invalid_data
  : public io_exception
{
public:
  /// Constructor
  invalid_data(std::string reason) MAPTK_NOTHROW;
  /// Destructor
  virtual ~invalid_data() MAPTK_NOTHROW;

  /// Reason the data is invalid
  std::string m_reason;
};


} // end maptk namespace

#endif // MAPTK_EXCEPTIONS_IO_H_
