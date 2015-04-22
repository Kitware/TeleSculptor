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

#include <maptk/c/error_handle.h>

#include <maptk/exceptions/base.h>
#include <maptk/logging_macros.h>


/// Macro allowing simpler population of an error handle
/**
 * Only does anything if error handle pointer is non-NULL.
 * \p msg should be a C string (char const*)
 */
#define POPULATE_EH(eh_ptr, ec, msg)                                         \
  do                                                                         \
  {                                                                          \
    if( reinterpret_cast<maptk_error_handle_t*>(eh_ptr) != NULL )            \
    {                                                                        \
      maptk_error_handle_t *PEH_eh_ptr_cast =                                \
        reinterpret_cast<maptk_error_handle_t*>(eh_ptr);                     \
      PEH_eh_ptr_cast->error_code = ec;                                      \
      PEH_eh_ptr_cast->message = (char*)malloc(sizeof(char) * strlen(msg));  \
      strcpy(PEH_eh_ptr_cast->message, msg);                                 \
    }                                                                        \
  } while(0)


/// Standardized try/catch for general use.
/**
 * If the provided code block contains a return, make sure to provide a
 * default/failure return after the use of the STANDARD_CATCH macro in case
 * an exception is thrown within the provided code block.
 *
 * Assuming \c eh_ptr points to an initialized maptk_error_handle_t instance.
   * An arbitrary catch sets a -1 error code and assignes to the message field
   * the same thing that is printed to logging statement.
   */
#define STANDARD_CATCH(log_prefix, eh_ptr, code)                \
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
        std::string msg = ss.str();                             \
        LOG_DEBUG( log_prefix, msg.c_str() );                   \
        POPULATE_EH( eh_ptr, -1, msg.c_str() );                 \
      }                                                         \
      catch( char const* e )                                    \
      {                                                         \
        std::ostringstream ss;                                  \
        ss << "Caught error message: " << e;                    \
        LOG_DEBUG( log_prefix, ss.str().c_str() );              \
        POPULATE_EH( eh_ptr, -1, ss.str().c_str() );            \
      }                                                         \
      catch(...)                                                \
      {                                                         \
        std::string msg("Caught other exception");              \
        LOG_DEBUG( log_prefix, msg );                           \
        POPULATE_EH( eh_ptr, -1, msg.c_str() );                 \
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
  typedef std::map< maptk_t*, size_t > ref_count_cache_t;

  /// Exception for when a given entry doesn't exist in this cache
  class NoEntryException
    : public maptk::maptk_core_base_exception
  {
  public:
    NoEntryException( std::string const &reason )
    {
      this->m_what = reason;
    }
  };

  /// Exception for when we're asked to do something with a null pointer
  class NullPointerException
    : public maptk::maptk_core_base_exception
  {
  public:
    NullPointerException( std::string const &reason)
    {
      this->m_what = reason;
    }
  };

  /// Constructor
  SharedPointerCache( std::string name )
    : cache_(),
      ref_count_cache_(),
      name_( name )
  {}

  /// Destructor
  virtual ~SharedPointerCache()
  {}

  /// Store a shared pointer
  void store( sptr_t sptr )
  {
    if( sptr.get() == NULL )
    {
      std::ostringstream ss;
      ss << get_log_prefix(sptr.get()) << ": Cannot store NULL pointer";
      throw NullPointerException(ss.str());
    }

    // If an sptr referencing the underlying pointer already exists in the map,
    // don't bother bashing the existing entry
    if( cache_.count( sptr.get() ) == 0 )
    {
      cache_[sptr.get()] = sptr;
      ref_count_cache_[sptr.get()] = 1;
    }
    else
    {
      ++ref_count_cache_[sptr.get()];
    }
  }

  /// Access a stored shared pointer based on a supplied pointer
  sptr_t get( maptk_t *ptr ) const
  {
    if( ptr == NULL )
    {
      std::ostringstream ss;
      ss << get_log_prefix(ptr) << ": Cannot get NULL pointer";
      throw NullPointerException(ss.str());
    }

    typename cache_t::const_iterator it = cache_.find( ptr );
    if( it != cache_.end() )
    {
      return it->second;
    }
    else
    {
      std::ostringstream ss;
      ss << get_log_prefix(ptr) << ": "
         << " No cached shared_ptr for the given pointer (ptr: " << ptr << ")";
      throw NoEntryException( ss.str() );
    }
  }

  /// Access a stored shared pointer based on the C interface opaque type
  sptr_t get( C_t *ptr ) const
  {
    return this->get( reinterpret_cast< maptk_t* >( ptr ) );
  }

  /// Erase an entry in the cache by maptk-type pointer
  void erase( maptk_t *ptr )
  {
    if( ptr == NULL )
    {
      std::ostringstream ss;
      ss << get_log_prefix(ptr) << ": Cannot erase NULL pointer";
      throw NullPointerException(ss.str());
    }

    typename cache_t::iterator c_it = cache_.find( ptr );
    if( c_it != cache_.end() )
    {
      --ref_count_cache_[ptr];
      // Only finally erase cache entry when store references reaches 0
      if( ref_count_cache_[ptr] <= 0 )
      {
        cache_.erase(c_it);
        ref_count_cache_.erase(ptr);
      }
    }
  }

  /// Erase an entry in the cache by C Interface opaque type pointer
  void erase( C_t *ptr )
  {
    return this->erase( reinterpret_cast< maptk_t* >( ptr ) );
  }

private:
  /// Cache of shared pointers for concrete instances
  cache_t cache_;
  /// Number of times an instance has been "stored" in this cache
  /**
   * This is basically cache local reference counting ensuring that the cache
   * only actually erases a caches sptr when the number erase calls equals the
   * number of store calls for a given instance pointer.
   */
  ref_count_cache_t ref_count_cache_;
  /// Name of cache
  std::string name_;

  /// Helper method to generate logging prefix string
  std::string get_log_prefix( maptk_t *ptr ) const
  {
    std::ostringstream ss;
    ss << "SharedPointerCache::" << this->name_ << "::" << ptr;
    return ss.str();
  }
};


/// Helper function to create a char** list of strings give a vector of strings
void make_string_list( std::vector<std::string> const &list,
                       unsigned int &length, char ** &strings );


} //end maptk_c namespace


#endif //MAPTK_C_HELPERS_C_UTILS_H_
