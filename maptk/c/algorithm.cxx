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

#include <sstream>

#include <maptk/algo/algorithm.h>

#include <maptk/c/helpers/algorithm.h>
#include <maptk/c/helpers/c_utils.h>

/// ImageIO stuff
#include <maptk/algo/image_io.h>
#include <maptk/c/helpers/image_container.h>


// ===========================================================================
// Helper stuff
// ---------------------------------------------------------------------------

namespace maptk_c
{

/// Global cache for all algorithm instances ever.
SharedPointerCache< maptk::algo::algorithm,
                    maptk_algorithm_t > ALGORITHM_SPTR_CACHE;

}


// ===========================================================================
// Functions on general algorithm pointer
// ---------------------------------------------------------------------------

char const* maptk_algorithm_type_name( maptk_algorithm_t *algo,
                                       maptk_error_handle_t *eh )
{
  STANDARD_CATCH(
    "C::algorithm::type_name", eh,

    return maptk_c::ALGORITHM_SPTR_CACHE
        .get( algo )->type_name().c_str();
  );
  return 0;
}


char const* maptk_algorithm_impl_name( maptk_algorithm_t *algo,
                                       maptk_error_handle_t *eh )
{
  STANDARD_CATCH(
    "C::algorithm::impl_name", eh,

    return maptk_c::ALGORITHM_SPTR_CACHE
        .get( algo )->impl_name().c_str();
  );
  return 0;
}


#define DEFINE_COMMON_ALGO_API( type )                                              \
  /* Make sptr cache for specific type */                                           \
  namespace maptk_c                                                                 \
  {                                                                                 \
    SharedPointerCache< maptk::algo::type,                                          \
                        maptk_algorithm_t > ALGORITHM_##type##_SPTR_CACHE;          \
  }                                                                                 \
  /* ==================================================================== */        \
  /* Functions on types (static methods)                                  */        \
  /* -------------------------------------------------------------------- */        \
  /* Create new instance of a specific algorithm implementation */                  \
  maptk_algorithm_t* maptk_algorithm_##type##_create( char const *impl_name )       \
  {                                                                                 \
    STANDARD_CATCH(                                                                 \
      "C::algorithm::" #type "::create", NULL,                                      \
      maptk::algo::type##_sptr algo_sptr = maptk::algo::type::create( impl_name );  \
      if( algo_sptr )                                                               \
      {                                                                             \
        maptk_c::ALGORITHM_SPTR_CACHE.store( algo_sptr );                           \
        maptk_c::ALGORITHM_##type##_SPTR_CACHE.store( algo_sptr );                  \
        return reinterpret_cast<maptk_algorithm_t*>(algo_sptr.get());               \
      }                                                                             \
    );                                                                              \
    return 0;                                                                       \
  }                                                                                 \
  /* Destroy an algorithm instance of this type */                                  \
  void maptk_algorithm_##type##_destroy( maptk_algorithm_t *algo,                   \
                                         maptk_error_handle_t *eh )                 \
  {                                                                                 \
    STANDARD_CATCH(                                                                 \
      "C::algorithm::" #type "::destroy", eh,                                       \
      maptk_c::ALGORITHM_SPTR_CACHE.erase( algo );                                  \
      maptk_c::ALGORITHM_##type##_SPTR_CACHE.erase( algo );                         \
    );                                                                              \
  }                                                                                 \
  /* Get a list of registered implementation names for this algorithm type */       \
  void maptk_algorithm_##type##_registered_names( unsigned int *length,             \
                                                  char ***names )                   \
  {                                                                                 \
    STANDARD_CATCH(                                                                 \
      "C::algorithm::" #type "::registered_names", NULL,                            \
      std::vector<std::string> name_list = maptk::algo::type::registered_names();   \
      maptk_c::make_string_list( name_list, *length, *names );                      \
    );                                                                              \
  }                                                                                 \
  /* ==================================================================== */        \
  /* Functions on algorithm instances                                     */        \
  /* -------------------------------------------------------------------- */        \
  /* Clone the given algorithm instance */                                          \
  maptk_algorithm_t* maptk_algorithm_##type##_clone( maptk_algorithm_t *algo )      \
  {                                                                                 \
    STANDARD_CATCH(                                                                 \
      "C::algorithm::" #type "::clone", NULL,                                       \
      maptk::algo::type##_sptr new_sptr =                                           \
        maptk_c::ALGORITHM_##type##_SPTR_CACHE.get( algo )->clone();                \
      if( new_sptr )                                                                \
      {                                                                             \
        maptk_c::ALGORITHM_SPTR_CACHE.store( new_sptr );                            \
        maptk_c::ALGORITHM_##type##_SPTR_CACHE.store( new_sptr );                   \
        return reinterpret_cast<maptk_algorithm_t*>(new_sptr.get());                \
      }                                                                             \
      else                                                                          \
      {                                                                             \
        throw "Failed to clone instance of type '" #type "'";                       \
      }                                                                             \
    );                                                                              \
    return 0;                                                                       \
  }


// ===========================================================================
// Algorithm: image_io
// ---------------------------------------------------------------------------

DEFINE_COMMON_ALGO_API( image_io );


/// Load image from file
maptk_image_container_t* maptk_algorithm_image_io_load( maptk_algorithm_t *algo,
                                                        char const *filename,
                                                        maptk_error_handle_t *eh )
{
  STANDARD_CATCH(
    "C::algorithm::image_io::load", eh,
    maptk::image_container_sptr ic_sptr =
        maptk_c::ALGORITHM_image_io_SPTR_CACHE.get( algo )->load( filename );
    maptk_c::IMGC_SPTR_CACHE.store( ic_sptr );
    return reinterpret_cast<maptk_image_container_t*>( ic_sptr.get() );
  );
  return 0;
}


/// Save an image to file
void maptk_algorithm_image_io_save( maptk_algorithm_t *algo,
                                    char const *filename,
                                    maptk_image_container_t *ic,
                                    maptk_error_handle_t *eh )
{
  STANDARD_CATCH(
    "C::algorithm::image_io::save", eh,
    maptk::image_container_sptr ic_sptr = maptk_c::IMGC_SPTR_CACHE.get(ic);
    maptk_c::ALGORITHM_image_io_SPTR_CACHE.get( algo )->save( filename, ic_sptr );
  );
}
