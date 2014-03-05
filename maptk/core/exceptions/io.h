/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief MAPTK Exceptions pertaining to IO operations
 */

#ifndef MAPTK_CORE_EXCEPTIONS_IO_H
#define MAPTK_CORE_EXCEPTIONS_IO_H

#include "base.h"
#include <string>
#include <maptk/core/types.h>

namespace maptk
{


/// MAPTK Generic IO exception
class MAPTK_CORE_EXPORT io_exception
  : public maptk_core_base_exception
{
public:
  /// Constructor
  io_exception() MAPTK_NOTHROW;
  /// Destructor
  virtual ~io_exception() MAPTK_NOTHROW;
};


/// Exception for when a given path doesn't point to anything on the filesystem
class MAPTK_CORE_EXPORT path_not_exists
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
class MAPTK_CORE_EXPORT path_not_a_file
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
class MAPTK_CORE_EXPORT path_not_a_directory
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
class MAPTK_CORE_EXPORT invalid_file
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
class MAPTK_CORE_EXPORT invalid_data
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

#endif // MAPTK_CORE_EXCEPTIONS_IO_H
