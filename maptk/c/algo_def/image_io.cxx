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
 * \brief C Interface to image_io algorithm implementation
 */

#include "image_io.h"

#include <maptk/algo/image_io.h>
#include <maptk/c/helpers/algorithm.h>
#include <maptk/c/helpers/image_container.h>


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
