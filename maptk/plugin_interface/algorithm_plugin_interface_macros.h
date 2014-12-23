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
 * \brief Algorithm definition type registration helper macros
 */

#ifndef _MAPTK_PLUGINS_INTERFACE_ALGORITHM_PLUGIN_INTERFACE_MACROS_H_
#define _MAPTK_PLUGINS_INTERFACE_ALGORITHM_PLUGIN_INTERFACE_MACROS_H_

#include <maptk/logging_macros.h>


// Helper macros for algorithm registration
/// Initialize required variable for algorithm type registration
/**
 * Side effect: Defines the integer variables ``_api_expected``,
 * ``_api_registered`` and ``_api_registrar``. Its probably not a good idea to
 * use these variable names in the current scope, unless expecting to
 * reference the ones defined here.
 *
 * \param reg The registrar we will be registering with.
 */
#define REGISTRATION_INIT( reg ) \
  int _api_expected = 0, _api_registered = 0; \
  maptk::registrar &_api_registrar = reg
/// Log to standard error a summary of registration results
/**
 * NOTE: Logging only occurs when build in debug (-DNDEBUG)
 */
#define REGISTRATION_SUMMARY() \
  LOG_DEBUG( "maptk::algorithm_plugin_interface_macros::REGISTRATION_SUMMARY", \
             "Registered " << _api_registered << " of " << _api_expected \
             << " algorithms" \
             << "\n\t(@" << __FILE__ << ")" )
/// Return the number of registrations that failed (int).
#define REGISTRATION_FAILURES() \
  (_api_expected - _api_registered)
/**
 * \brief Given a maptk::algorithm_def type, attempt registration with the
 *        given registrar
 * \param type Algorithm definition type
 */
#define REGISTER_TYPE( type ) \
  ++_api_expected; \
  _api_registered += type::register_self( _api_registrar )


#endif
