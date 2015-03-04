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
 * \brief C Interface to track_features algorithm implementation
 */

#include "track_features.h"

#include <maptk/algo/track_features.h>

#include <maptk/c/helpers/algorithm.h>
#include <maptk/c/helpers/image_container.h>
#include <maptk/c/helpers/track_set.h>


/// Common API implementation
DEFINE_COMMON_ALGO_API( track_features );


/// Extend a previous set of tracks using the current frame
maptk_trackset_t*
maptk_algorithm_track_features_track( maptk_algorithm_t *algo,
                                      maptk_trackset_t *prev_tracks,
                                      unsigned int frame_num,
                                      maptk_image_container_t *ic,
                                      maptk_error_handle_t *eh )
{
  STANDARD_CATCH(
    "C::algorithm::track_features::track", eh,
    using namespace maptk_c;
    maptk::track_set_sptr ts_sptr =
      ALGORITHM_track_features_SPTR_CACHE.get( algo )->track(
        TRACK_SET_SPTR_CACHE.get( prev_tracks ),
        frame_num,
        IMGC_SPTR_CACHE.get( ic )
      );
    TRACK_SET_SPTR_CACHE.store( ts_sptr );
    return reinterpret_cast<maptk_trackset_t*>( ts_sptr.get() );
  );
  return 0;
}


/// Extend a previous set of tracks using the current frame, masked version
maptk_trackset_t*
maptk_algorithm_track_features_track_with_mask( maptk_algorithm_t *algo,
                                                maptk_trackset_t *prev_tracks,
                                                unsigned int frame_num,
                                                maptk_image_container_t *ic,
                                                maptk_image_container_t *mask,
                                                maptk_error_handle_t *eh )
{
  STANDARD_CATCH(
    "C::algorithm::track_features::track", eh,
    using namespace maptk_c;
    maptk::track_set_sptr ts_sptr =
      ALGORITHM_track_features_SPTR_CACHE.get( algo )->track(
        TRACK_SET_SPTR_CACHE.get( prev_tracks ),
        frame_num,
        IMGC_SPTR_CACHE.get( ic ),
        IMGC_SPTR_CACHE.get( mask )
      );
    TRACK_SET_SPTR_CACHE.store( ts_sptr );
    return reinterpret_cast<maptk_trackset_t*>( ts_sptr.get() );
  );
  return 0;
}
