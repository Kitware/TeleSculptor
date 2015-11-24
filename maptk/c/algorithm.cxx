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

#include <cstdlib>
#include <string>

#include <boost/foreach.hpp>

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
// Static algorithm methods
//
// These methods usually require one or more of the algorithm type and
// implementation labels.
// ---------------------------------------------------------------------------


/// Check the given type label against registered algorithm types
bool
maptk_algorithm_has_type_name( char const *type_name,
                               maptk_error_handle_t *eh )
{
  STANDARD_CATCH(
    "C::algorithm::has_type_name", eh,
    return maptk::algo::algorithm::has_type_name( type_name );
  );
  return false;
}


/// Check the given type and implementation names against registered algorithms
bool
maptk_algorithm_has_impl_name( char const *type_name,
                               char const *impl_name,
                               maptk_error_handle_t *eh )
{
  STANDARD_CATCH(
    "C::algorithm::has_impl_name", eh,
    return maptk::algo::algorithm::has_impl_name( type_name,
                                                  impl_name );
  );
  return false;
}


/// Return an array of impl names of registered algorithms of the given type
void
maptk_algorithm_registered_names( char const *type_name,
                                  unsigned int *length,
                                  char ***names,
                                  maptk_error_handle_t *eh )
{
  char const *prefix = "C::algorithm::registered_names";
  (void)prefix; // silence unused variable warnings
  STANDARD_CATCH(
    prefix, eh,

    std::vector<std::string> name_list =
      maptk::algo::algorithm::registered_names(type_name);
    maptk_c::make_string_list( name_list, *length, *names );
    return;

  );
  // Failure case
  *length = 0;
  *names = NULL;
}


/// Create a new algorithm instance of the requested type and implementation
maptk_algorithm_t*
maptk_algorithm_create( char const *type_name,
                        char const *impl_name,
                        maptk_error_handle_t *eh )
{
  STANDARD_CATCH(
    "C::algorithm::create", eh,

    maptk::algo::algorithm_sptr algo_sptr =
      maptk::algo::algorithm::create(type_name, impl_name);
    maptk_c::ALGORITHM_SPTR_CACHE.store( algo_sptr );
    return reinterpret_cast<maptk_algorithm_t*>(algo_sptr.get());

  );
  return NULL;
}


/// Helper function for properly getting a nested algorithm's configuration
void
maptk_algorithm_get_nested_algo_configuration(char const *type_name,
                                              char const *name,
                                              maptk_config_block_t *cb,
                                              maptk_algorithm_t *algo,
                                              maptk_error_handle_t *eh )
{
  STANDARD_CATCH(
    "C::algorithm::get_nested_algo_configuration", eh,

    // Use an empty sptr unless we were given a non-null algorithm pointer.
    maptk::algo::algorithm_sptr algo_sptr;
    if( algo != NULL )
    {
      algo_sptr = maptk_c::ALGORITHM_SPTR_CACHE.get( algo );
    }

    maptk::algo::algorithm::get_nested_algo_configuration(
      type_name, name,
      maptk_c::CONFIG_BLOCK_SPTR_CACHE.get( cb ),
      algo_sptr
      );

  );
}


/// Helper function for properly setting a nested algorithm's configuration
void
maptk_algorithm_set_nested_algo_configuration( char const *type_name,
                                               char const *name,
                                               maptk_config_block_t *cb,
                                               maptk_algorithm_t **algo,
                                               maptk_error_handle_t *eh )
{
  STANDARD_CATCH(
    "C::algorithm::set_nested_algo_configuration", eh,

    // Always start with an empty sptr so we know when an instance was created
    // or not.
    maptk::algo::algorithm_sptr algo_sptr;

    maptk::algo::algorithm::set_nested_algo_configuration(
      type_name, name,
      maptk_c::CONFIG_BLOCK_SPTR_CACHE.get( cb ),
      algo_sptr
      );

    // Set algo pointer if an algo instance was generated from the parent
    // function.
    if( algo_sptr.get() )
    {
      maptk_c::ALGORITHM_SPTR_CACHE.store( algo_sptr );
      *algo = reinterpret_cast<maptk_algorithm_t*>(algo_sptr.get());
    }

  );
}


/// Helper function for checking that basic nested algorithm configuration validity
bool
maptk_algorithm_check_nested_algo_configuration( char const *type_name,
                                                 char const *name,
                                                 maptk_config_block_t *cb,
                                                 maptk_error_handle_t *eh )
{
  STANDARD_CATCH(
    "C::algorithm::check_nested_algo_configuration", eh,

    return maptk::algo::algorithm::check_nested_algo_configuration(
      type_name, name,
      maptk_c::CONFIG_BLOCK_SPTR_CACHE.get( cb )
      );

  );
  return false;
}


// ===========================================================================
// Functions on general algorithm pointer
// ---------------------------------------------------------------------------

maptk_string_t*
maptk_algorithm_type_name( maptk_algorithm_t *algo,
                           maptk_error_handle_t *eh )
{
  STANDARD_CATCH(
    "C::algorithm::type_name", eh,
    std::string s( maptk_c::ALGORITHM_SPTR_CACHE.get( algo )->type_name() );
    return maptk_string_new( s.length(), s.c_str() );
  );
  return 0;
}


maptk_string_t*
maptk_algorithm_impl_name( maptk_algorithm_t *algo,
                           maptk_error_handle_t *eh )
{
  STANDARD_CATCH(
    "C::algorithm::impl_name", eh,
    std::string s( maptk_c::ALGORITHM_SPTR_CACHE.get( algo )->impl_name() );
    return maptk_string_new( s.length(), s.c_str() );
  );
  return 0;
}


/// Return optional descriptive string about an implementation
maptk_string_t*
maptk_algorithm_description( maptk_algorithm_t *algo,
                             maptk_error_handle_t *eh )
{
  STANDARD_CATCH(
    "C::algorithm::description", eh,
    std::string s( maptk_c::ALGORITHM_SPTR_CACHE.get( algo )->description() );
    return maptk_string_new( s.length(), s.c_str() );
  );
  return NULL;
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


/// Clone the given algorithm instance
maptk_algorithm_t*
maptk_algorithm_clone( maptk_algorithm_t *algo,
                       maptk_error_handle_t *eh )
{
  STANDARD_CATCH(
    "C::algorithm::clone", eh,
    if( algo == NULL )
    {
      return NULL;
    }
    maptk::algo::algorithm_sptr clone_sptr =
      maptk_c::ALGORITHM_SPTR_CACHE.get( algo )->base_clone();
    maptk_c::ALGORITHM_SPTR_CACHE.store( clone_sptr );
    return reinterpret_cast<maptk_algorithm_t*>( clone_sptr.get() );

  );
  return 0;
}


/// Destroy the given algorithm instance
void
maptk_algorithm_destroy( maptk_algorithm_t *algo,
                         maptk_error_handle_t *eh )
{
  STANDARD_CATCH(
    "C::algorithm::destroy", eh,
    maptk_c::ALGORITHM_SPTR_CACHE.erase( algo );
  );
}
