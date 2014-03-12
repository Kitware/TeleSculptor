/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief MAPTK base exception implementation
 */

#include "base.h"

namespace maptk
{

maptk_core_base_exception
::maptk_core_base_exception() MAPTK_NOTHROW
  : std::exception()
{
}

maptk_core_base_exception
::~maptk_core_base_exception() MAPTK_NOTHROW
{
}

char const*
maptk_core_base_exception
::what() const MAPTK_NOTHROW
{
  return this->m_what.c_str();
}

invalid_value
::invalid_value(std::string reason) MAPTK_NOTHROW
  : m_reason(reason)
{
  m_what = "Invalid value(s): " + reason;
}

invalid_value
::~invalid_value() MAPTK_NOTHROW
{
}

} // end namespace maptk
