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
 * \brief MAPTK base exception interface
 */

#ifndef MAPTK_CORE_EXCEPTIONS_BASE_H
#define MAPTK_CORE_EXCEPTIONS_BASE_H

#include <maptk/core/core_config.h>
#include <string>

namespace maptk
{

/// The base class for all maptk/core exceptions
/**
 * \ingroup exceptions
 */
class MAPTK_CORE_EXPORT maptk_core_base_exception
  : public std::exception
{
  public:
    /// Constructor
    maptk_core_base_exception() MAPTK_NOTHROW;
    /// Destructor
    virtual ~maptk_core_base_exception() MAPTK_NOTHROW;

    /// Description of the exception
    /**
     * \returns A string describing what went wrong.
     */
    char const* what() const MAPTK_NOTHROW;
  protected:
    /// descriptive string as to what happened to cause the exception.
    std::string m_what;
};

/// Exception for incorrect input values
/**
 * \ingroup exceptions
 */
class MAPTK_CORE_EXPORT invalid_value
  : public maptk_core_base_exception
{
public:
  /// Constructor
  invalid_value(std::string reason) MAPTK_NOTHROW;
  /// Destructor
  virtual ~invalid_value() MAPTK_NOTHROW;
protected:
  /// Reason for invalidity
  std::string m_reason;
};

} // end namespace maptk

#endif // MAPTK_CORE_EXCEPTIONS_BASE_H
