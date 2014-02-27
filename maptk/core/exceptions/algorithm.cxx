/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief algorithm exception implementations
 */

#include "algorithm.h"
#include <sstream>

namespace maptk
{

algorithm_exception
::algorithm_exception(std::string type,
                      std::string impl,
                      std::string reason) MAPTK_NOTHROW
  : m_algo_type(type)
  , m_algo_impl(impl)
  , m_reason(reason)
{
  // generic what string
  std::ostringstream sstr;
  sstr << "[algo::" << type << "::" << impl << "]: "
       << reason;
  m_what = sstr.str();
}

algorithm_exception
::~algorithm_exception() MAPTK_NOTHROW
{
}

algorithm_configuration_exception
::algorithm_configuration_exception(std::string type,
                                    std::string impl,
                                    std::string reason) MAPTK_NOTHROW
  : algorithm_exception(type, impl, reason)
{
  std::ostringstream sstr;
  sstr << "Failed to configure algorithm "
       << "\"" << m_algo_type << "::" << m_algo_impl << "\" due to: "
       << reason;
  m_what = sstr.str();
}

algorithm_configuration_exception
::~algorithm_configuration_exception() MAPTK_NOTHROW
{
}

invalid_name_exception
::invalid_name_exception(std::string type,
                         std::string impl) MAPTK_NOTHROW
  : algorithm_exception(type, impl, "")
{
  std::ostringstream sstr;
  sstr << "Invalid algorithm impl name \"" << m_algo_impl << "\""
       << "for type \"" << m_algo_type << "\".";
  m_what = sstr.str();
}

invalid_name_exception
::~invalid_name_exception() MAPTK_NOTHROW
{
}


} // end namespace maptk
