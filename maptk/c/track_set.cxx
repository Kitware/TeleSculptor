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
 * \brief C Interface to maptk::track_set implementation
 */

#include "track_set.h"

#include <vector>

#include <maptk/logging_macros.h>
#include <maptk/track_set.h>
#include <maptk/track_set_io.h>

#include <maptk/c/helpers/c_utils.h>
#include <maptk/c/helpers/track.h>
#include <maptk/c/helpers/track_set.h>


namespace maptk_c
{

SharedPointerCache< maptk::track_set, maptk_trackset_t >
  TRACK_SET_SPTR_CACHE( "track_set" );

}


/// Create a new track set from an array of track instances
maptk_trackset_t*
maptk_trackset_new( size_t length, maptk_track_t **tracks )
{
  STANDARD_CATCH(
    "C::track_set::new", NULL,
    using namespace maptk_c;

    std::vector<maptk::track_sptr> track_vec;
    for( size_t i=0; i < length; ++i )
    {
      track_vec.push_back( TRACK_SPTR_CACHE.get( tracks[i] ) );
    }
    maptk::track_set_sptr ts_sptr(
      new maptk::simple_track_set( track_vec )
    );
    TRACK_SET_SPTR_CACHE.store( ts_sptr );
    return reinterpret_cast<maptk_trackset_t*>( ts_sptr.get() );
  );
  return 0;
}


/// Create a new track set as read from file
maptk_trackset_t*
maptk_trackset_new_from_file( char const *filepath,
                              maptk_error_handle_t *eh )
{
  STANDARD_CATCH(
    "C::track_set::new_from_file", eh,
    maptk::track_set_sptr ts_sptr( maptk::read_track_file( filepath ) );
    maptk_c::TRACK_SET_SPTR_CACHE.store( ts_sptr );
    return reinterpret_cast<maptk_trackset_t*>( ts_sptr.get() );
  );
  return 0;
}


/// Destroy a track set instance
void
maptk_trackset_destroy( maptk_trackset_t *track_set,
                        maptk_error_handle_t *eh )
{
  STANDARD_CATCH(
    "C::track_set::destroy", eh,
    maptk_c::TRACK_SET_SPTR_CACHE.erase( track_set );
  );
}


/// Get the size of the track set
size_t
maptk_trackset_size( maptk_trackset_t *track_set,
                     maptk_error_handle_t *eh )
{
  STANDARD_CATCH(
    "C::track_set::size", eh,
    return maptk_c::TRACK_SET_SPTR_CACHE.get( track_set )->size();
  );
  return 0;
}


/// Write track set to the given filepath
MAPTK_C_EXPORT
void
maptk_trackset_write_track_file( maptk_trackset_t* ts,
                                 char const *filepath,
                                 maptk_error_handle_t *eh )
{
  STANDARD_CATCH(
    "C::track_set::write_track_file", eh,
    maptk::write_track_file(
      maptk_c::TRACK_SET_SPTR_CACHE.get( ts ),
      filepath
    );
  );
}
