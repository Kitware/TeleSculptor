/*ckwg +29
 * Copyright 2014-2015 by Kitware, Inc.
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

#ifndef MAPTK_PLUGIN_INTERFACE_ALGORITHM_PLUGIN_INTERFACE_MACROS_H_
#define MAPTK_PLUGIN_INTERFACE_ALGORITHM_PLUGIN_INTERFACE_MACROS_H_

#include <iostream>
#include <vital/registrar.h>


// Helper macros for algorithm registration
/// Initialize required variable for algorithm type registration
/**
 * Side effect: Defines the integer variables ``maptk_api_expected_``,
 * ``maptk_api_registered_`` and ``maptk_api_registrar_``. Its probably not a good
 * idea to use these variable names in the current scope, unless expecting to
 * reference the ones defined here.
 *
 * \param reg The registrar we will be registering with.
 */
#define REGISTRATION_INIT( reg ) \
  unsigned int maptk_api_expected_ = 0, maptk_api_registered_ = 0; \
  vital::registrar &maptk_api_registrar_ = reg


/// Log to standard error a summary of registration results
/**
 * NOTE: Logging only occurs when built in debug (-DNDEBUG)
 */
#ifndef NDEBUG
# define REGISTRATION_SUMMARY() \
    std::cerr << "[DEBUG][maptk::algorithm_plugin_interface_macros::REGISTRATION_SUMMARY] " \
              << "Registered " << maptk_api_registered_ << " of " << maptk_api_expected_ << " algorithms" << std::endl \
              << "\t(@" << __FILE__ << ")" << std::endl;
#else
# define REGISTRATION_SUMMARY()
#endif


/// Return the number of registrations that failed (int).
#define REGISTRATION_FAILURES() \
  (maptk_api_expected_ - maptk_api_registered_)


/**
 * \brief Given a maptk::algorithm_def type, attempt registration with the
 *        given registrar
 * \param type Algorithm definition type
 */
#define REGISTER_TYPE( type ) \
  ++maptk_api_expected_; \
  maptk_api_registered_ += type::register_self( maptk_api_registrar_ )


#endif // MAPTK_PLUGIN_INTERFACE_ALGORITHM_PLUGIN_INTERFACE_MACROS_H_
