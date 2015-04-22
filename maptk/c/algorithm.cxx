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
 * \brief C interface implementation of base algorithm/_def/_impl classes
 */

#include "algorithm.h"

#include <maptk/algo/algorithm.h>

#include <maptk/c/helpers/algorithm.h>
#include <maptk/c/helpers/c_utils.h>


// ===========================================================================
// Helper stuff
// ---------------------------------------------------------------------------

namespace maptk_c
{

/// Global cache for all algorithm instances ever.
SharedPointerCache< maptk::algo::algorithm,
                    maptk_algorithm_t > ALGORITHM_SPTR_CACHE( "algorithm" );

}


// ===========================================================================
// Functions on general algorithm pointer
// ---------------------------------------------------------------------------

maptk_string_t* maptk_algorithm_type_name( maptk_algorithm_t *algo,
                                           maptk_error_handle_t *eh )
{
  STANDARD_CATCH(
    "C::algorithm::type_name", eh,
    std::string s( maptk_c::ALGORITHM_SPTR_CACHE.get( algo )->type_name() );
    return maptk_string_new( s.length(), s.c_str() );
  );
  return 0;
}


maptk_string_t* maptk_algorithm_impl_name( maptk_algorithm_t *algo,
                                           maptk_error_handle_t *eh )
{
  STANDARD_CATCH(
    "C::algorithm::impl_name", eh,
    std::string s( maptk_c::ALGORITHM_SPTR_CACHE.get( algo )->impl_name() );
    return maptk_string_new( s.length(), s.c_str() );
  );
  return 0;
}


/// Get an algorithm implementation's configuration block
maptk_config_block_t*
maptk_algorithm_get_impl_configuration( maptk_algorithm_t *algo,
                                        maptk_error_handle_t *eh )
{
  STANDARD_CATCH(
    "C::algorithm::get_impl_configuration", eh,
    maptk::config_block_sptr cb_sptr =
      maptk_c::ALGORITHM_SPTR_CACHE.get( algo )->get_configuration();
    maptk_c::CONFIG_BLOCK_SPTR_CACHE.store( cb_sptr );
    return reinterpret_cast<maptk_config_block_t*>( cb_sptr.get() );
  );
  return 0;
}


/// Set this algorithm implementation's properties via a config block
void
maptk_algorithm_set_impl_configuration( maptk_algorithm_t *algo,
                                        maptk_config_block_t *cb,
                                        maptk_error_handle_t *eh )
{
  STANDARD_CATCH(
    "C::algorithm::set_impl_configuration", eh,
    maptk_c::ALGORITHM_SPTR_CACHE.get( algo )->set_configuration(
      maptk_c::CONFIG_BLOCK_SPTR_CACHE.get( cb )
    );
  );
}


/// Check that the algorithm implementation's configuration is valid
bool
maptk_algorithm_check_impl_configuration( maptk_algorithm_t *algo,
                                          maptk_config_block_t *cb,
                                          maptk_error_handle_t *eh )
{
  STANDARD_CATCH(
    "C::algorithm::check_impl_configuration", eh,
    maptk_c::ALGORITHM_SPTR_CACHE.get( algo )->check_configuration(
      maptk_c::CONFIG_BLOCK_SPTR_CACHE.get( cb )
    );
  );
  return false;
}
