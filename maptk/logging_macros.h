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
 * \brief Simple convenience macros for STDERR logging messages.
 */


#ifndef _MAPTK_LOGGING_MACROS_H_
#define _MAPTK_LOGGING_MACROS_H_


/// Logging / Debugging helper macros
#ifndef NDEBUG
/// Display a debugging message
# define LOG_DEBUG(prefix, msg) \
  std::cerr << "[DEBUG][" << prefix << "] " << msg << std::endl
/// Execute debug code
# define DEBUG_CODE(code) code
#else
/// Display a debugging message
# define LOG_DEBUG(prefix, msg)
/// Execute debug code
# define DEBUG_CODE(code)
#endif

/// Display an informational message
#define LOG_INFO(prefix, msg) \
  std::cerr << " [INFO][" << prefix << "] " << msg << std::endl

/// Display a warning message
#define LOG_WARN(prefix, msg) \
  std::cerr << " [WARN][" << prefix << "] " << msg << std::endl
#define LOG_WARNING(prefix, msg) \
  LOG_WARN(prefix, msg)

/// Display an error message
#define LOG_ERROR(prefix, msg) \
  std::cerr << "[ERROR][" << prefix << "] " << msg << std::endl


#endif // _MAPTK_LOGGING_MACROS_H_
