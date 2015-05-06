/*ckwg +29
 * Copyright 2015 by Kitware, Inc.
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
 * \brief C++ Helper utilities for C interface of MAPTK algorithms
 *
 * Private header for use in cxx implementation files.
 */

#ifndef MAPTK_C_HELPERS_ALGORITHM_H_
#define MAPTK_C_HELPERS_ALGORITHM_H_

#include <sstream>
#include <string>

#include <maptk/algo/algorithm.h>
#include <maptk/exceptions/algorithm.h>

#include <maptk/c/algorithm.h>
#include <maptk/c/helpers/c_utils.h>
#include <maptk/c/helpers/config_block.h>


namespace maptk_c
{

extern SharedPointerCache< maptk::algo::algorithm,
                           maptk_algorithm_t > ALGORITHM_SPTR_CACHE;


class invalid_algorithm_pointer
  : public maptk::maptk_core_base_exception
{
public:
  invalid_algorithm_pointer( std::string reason )
  {
    this->m_what = reason;
  }
};


}


/// Wrapper for base standard catch to be used in implementation of algo def methods
/**
 * Functions for specific algorithm definitions take generic
 * \p maptk_algorithm_t pointers. This extended macro takes additional
 * parameters that ensure the algororithm instance provided to the function
 * is of an appropriate type, erroring when it is not. This prevents unintended
 * segfaults from occuring and leaving a more informative error trail.
 */
#define ALGO_STANDARD_CATCH(log_prefix, eh_ptr, algo_ptr, expected_type, code)  \
  STANDARD_CATCH( log_prefix, eh_ptr,                                           \
    std::string algo_ptr_type =                                                 \
      maptk_c::ALGORITHM_SPTR_CACHE.get( algo_ptr )->type_name();               \
    if( algo_ptr_type != expected_type )                                        \
    {                                                                           \
      std::ostringstream ss;                                                    \
      ss << "Given algorithm of type '" << algo_ptr_type << "' "                \
         << "when we are expecting type '" << expected_type << "'";             \
      throw maptk_c::invalid_algorithm_pointer( ss.str() );                     \
    }                                                                           \
    code                                                                        \
  );


#endif // MAPTK_C_HELPERS_ALGORITHM_H_
