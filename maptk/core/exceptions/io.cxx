/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
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


} // end maptk namespace
