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
 * \brief Common C Interface Utilities
 *
 * These utilities should only be used in CXX implementation files due to
 * their use of C++ structures.
 */

#ifndef MAPTK_C_HELPERS_C_UTILS_H_
#define MAPTK_C_HELPERS_C_UTILS_H_

#include <cstdlib>
#include <cstring>
#include <exception>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <boost/shared_ptr.hpp>

#include <maptk/exceptions/base.h>
#include <maptk/logging_macros.h>


/// Standardized try/catch for general use.
/**
 * If the provided code block contains a return, make sure to provide a
 * default/failure return after the use of the STANDARD_CATCH macro in case
 * an exception is thrown within the provided code block.
 */
#define STANDARD_CATCH(log_prefix, code)                      \
  do                                                          \
  {                                                           \
    try                                                       \
    {                                                         \
      code                                                    \
    }                                                         \
    catch( std::exception const &e )                          \
    {                                                         \
      std::ostringstream ss;                                  \
      ss << "Caught exception in C interface: " << e.what();  \
      LOG_WARN( log_prefix, ss.str().c_str() );               \
    }                                                         \
  } while( 0 )


namespace maptk_c
{


/// Common shared pointer cache object
template < typename maptk_t,  // MAPTK type
           typename C_t >     // C Interface opaque type
class SharedPointerCache
{
public:
  typedef boost::shared_ptr< maptk_t > sptr_t;
  typedef std::map< maptk_t*, sptr_t > cache_t;

  /// Exception for when a given entry doesn't exist in this cache
  class NoEntryException
    : public maptk::maptk_core_base_exception
  {
  public:
    NoEntryException( std::string reason )
    {
      this->m_what = reason;
    }
  };

  /// Constructor
  SharedPointerCache()
    : cache_()
  {}

  /// Destructor
  virtual ~SharedPointerCache()
  {}

  /// Store a shared pointer
  void store( sptr_t sptr )
  {
    // If an sptr referencing the underlying pointer already exists in the map,
    // don't bother bashing the existing entry
    if( cache_.count( sptr.get() ) == 0 )
    {
      cache_[sptr.get()] = sptr;
    }
  }

  /// Access a stored shared pointer based on a supplied pointer
  sptr_t get( maptk_t *ptr ) const
  {
    typename cache_t::const_iterator it = cache_.find( ptr );
    if( it != cache_.end() )
    {
      return it->second;
    }
    else
    {
      std::ostringstream ss;
      ss << "No cached shared_ptr for the given pointer "
         << "(ptr: " << ptr << ")";
      throw NoEntryException( ss.str() );
    }
  }

  /// Access a stored shared pointer based on the C interface opaque type
  sptr_t get( C_t *ptr ) const
  {
    return this->get( reinterpret_cast< maptk_t* >( ptr ) );
  }

  /// Erase an entry in the cache by maptk-type pointer
  /**
   * \returns 1 if an entry was erased and 0 if there wasn't.
   */
  unsigned int erase( maptk_t *ptr )
  {
    return cache_.erase( ptr );
  }

  /// Erase an entry in the cache by C Interface opaque type pointer
  unsigned int erase( C_t *ptr )
  {
    return this->erase( reinterpret_cast< maptk_t* >( ptr ) );
  }

private:
  cache_t cache_;
};


/// Helper function to create a char** list of strings give a vector of strings
void make_string_list( std::vector<std::string> const &list,
                       unsigned int &length, char ** &strings );



}



#endif //MAPTK_C_HELPERS_C_UTILS_H_
