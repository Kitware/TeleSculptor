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
 * \brief C Interface to \p config_block object implementation
 */


#include "config_block.h"

#include <map>
#include <sstream>
#include <string>

#include <maptk/config_block.h>
#include <maptk/exceptions.h>
#include <maptk/logging_macros.h>

#include <maptk/c/c_utils.h>


namespace //anonymous
{

/// Since the interface to maptk::config block uses Boost shared pointers, keep
/// a cached reference to the shared pointer upon creation of the config_block,
/// removing the reference upon deletion.
typedef std::map<maptk::config_block*, maptk::config_block_sptr> cb_cache_t;
static cb_cache_t CB_SPTR_CACHE;


/// Exception for when entry not found in cache map
class cb_cache_exception
  : public maptk::maptk_core_base_exception
{
public:
  cb_cache_exception(std::string reason)
  {
    this->m_what = reason;
  }
};


/// Standardized access to a maptk::config_block_sptr in the cache
/**
 * The code segment is only executed if the given config_block pointer exists
 * in the map
 *
 * Within the code segment, the variable ``cb_sptr`` of type
 * maptk::config_block_sptr is defined and points to the accessed
 * maptk::config_block.
 */
#define ACCESS_CB(cb_p, code)                                             \
  do                                                                      \
  {                                                                       \
    cb_cache_t::iterator it =                                             \
      CB_SPTR_CACHE.find( reinterpret_cast<maptk::config_block*>(cb_p) ); \
    if( it != CB_SPTR_CACHE.end() )                                       \
    {                                                                     \
      maptk::config_block_sptr cb_sptr = it->second;                      \
      code                                                                \
    }                                                                     \
    else                                                                  \
    {                                                                     \
      throw cb_cache_exception("No cached config_block by the given "     \
                               "pointer.");                               \
    }                                                                     \
  } while( 0 )


} //end anonymous namespace


/// Create a new, empty \p config_block object
config_block_s* maptk_config_block_new()
{
  STANDARD_CATCH(
    "maptk::config_block::new",

    return maptk_config_block_new_named("");

  );
  return 0;
}

/// Create a new, empty \p config_block object with a name
config_block_s* maptk_config_block_new_named( char const *name )
{
  STANDARD_CATCH(
    "maptk::config_block::new_named",

    maptk::config_block_sptr cb_sptr = maptk::config_block::empty_config( name );
    CB_SPTR_CACHE[cb_sptr.get()] = cb_sptr;
    return reinterpret_cast<config_block_s*>(cb_sptr.get());

  );
  return 0;
}

/// Destroy a config block object
unsigned int maptk_config_block_destroy( config_block_s *cb )
{
  STANDARD_CATCH(
    "maptk::config_block::destroy",

    return CB_SPTR_CACHE.erase( reinterpret_cast<maptk::config_block*>(cb) );

  );
  return 0;
}

/// Get the name of the \p config_block instance
char const* maptk_config_block_get_name( config_block_s *cb )
{
  STANDARD_CATCH(
    "maptk::config_block::get_name",

    ACCESS_CB(cb,
      return cb_sptr->get_name().c_str();
    );

  );
  return "";
}

/// Get a copy of a sub-block of the configuration
config_block_s* maptk_config_block_subblock( config_block_s *cb,
                                             char const *key )
{
  STANDARD_CATCH(
    "maptk::config_block::subblock",

    ACCESS_CB( cb,
      maptk::config_block_sptr sb_sptr = cb_sptr->subblock( key );
      CB_SPTR_CACHE[sb_sptr.get()] = sb_sptr;
      return reinterpret_cast<config_block_s*>( sb_sptr.get() );
    );

  );
  return 0;
}

/// Get a mutable view of a sub-block within a configuration
config_block_s* maptk_config_block_subblock_view( config_block_s *cb,
                                                  char const *key )
{
  STANDARD_CATCH(
    "maptk::config_block::subblock_view",

    ACCESS_CB( cb,
      maptk::config_block_sptr sb_sptr = cb_sptr->subblock_view( key );
      CB_SPTR_CACHE[sb_sptr.get()] = sb_sptr;
      return reinterpret_cast<config_block_s*>( sb_sptr.get() );
    );

  );
  return 0;
}

///// Get the string value for a key
//char* maptk_config_block_get_value( config_block_s *cb,
//                                    char* key )
//{
//  return reinterpret_cast<maptk::config_block*>(cb)->get_value<std::string>( key ).c_str();
//}
//
///// Get the string value for a key if it exists, else the default
//char*  maptk_config_block_get_value_default( config_block_s *cb,
//                                             char* key,
//                                             char* default )
//{
//  return reinterpret_cast<maptk::config_block*>(cb)->get_value<std::string>(key, default).c_str();
//}
//
///// Get the description string for a given key
//char* maptk_config_block_get_description( config_block_s *cb,
//                                          char* key )
//{
//  return reinterpret_cast<maptk::config_block*>(cb)->get_description( key ).c_str();
//}
//
///// Set the string value for a key
//void maptk_config_block_set_value( config_block_s *cb,
//                                   char *key,
//                                   char *value )
//{
//  reinterpret_cast<maptk::config_block*>(cb)->set_value( key, value );
//}
//
///// Merge another \p config_block's entries into this \p config_block
//void maptk_config_block_merge_config( config_block_s *cb,
//                                      config_block_s *other )
//{
//  reinterpret_cast<maptk::config_block*>(cb)->merge( *reinterpret_cast<maptk::config_block*>(other) );
//}
//
///// Check if a value exists for the given key
//int maptk_config_block_has_key( config_block_s *cb,
//                                char *key );
