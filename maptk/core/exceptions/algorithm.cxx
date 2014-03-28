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
