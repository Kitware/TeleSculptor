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
 * \brief maptk::image C interface implementation
 */

#include "image.h"

#include <maptk/image.h>

#include <maptk/c/helpers/c_utils.h>


/// Create a new, empty image
maptk_image_t* maptk_image_new()
{
  STANDARD_CATCH(
    "C::image:new", 0,
    return reinterpret_cast<maptk_image_t*>( new maptk::image() );
  );
  return 0;
}


/// Create a new image with dimensions, allocating memory
maptk_image_t* maptk_image_new_with_dim( size_t width, size_t height,
                                         size_t depth, bool interleave )
{
  STANDARD_CATCH(
    "C::image:new_with_dim", 0,
    return reinterpret_cast<maptk_image_t*>(
        new maptk::image( width, height, depth, interleave )
      );
  );
  return 0;
}


/// Create a new image from existing data
maptk_image_t* maptk_image_new_from_data( unsigned char const *first_pixel,
                                          size_t width, size_t height,
                                          size_t depth, ptrdiff_t w_step,
                                          ptrdiff_t h_step, ptrdiff_t d_step )
{
  STANDARD_CATCH(
    "C::image::new_from_data", 0,
    return reinterpret_cast<maptk_image_t*>(
        new maptk::image( first_pixel, width, height, depth,
                          w_step, h_step, d_step )
      );
  );
  return 0;
}


/// Create a new image from an existing image
maptk_image_t* maptk_image_new_from_image( maptk_image_t *other_image )
{
  STANDARD_CATCH(
    "C::image::new_from_data", 0,
    return reinterpret_cast<maptk_image_t*>(
        new maptk::image( *reinterpret_cast<maptk::image*>(other_image) )
      );
  );
  return 0;
}


/// Destroy an image instance
void maptk_image_destroy( maptk_image_t *image )
{
  STANDARD_CATCH(
    "C::image::desroy", 0,
    delete reinterpret_cast<maptk::image*>( image );
  );
};


/// Get the number of bytes allocated in the given image
size_t maptk_image_size( maptk_image_t *image )
{
  STANDARD_CATCH(
    "C::image::size", 0,
    return reinterpret_cast<maptk::image*>(image)->size();
  );
  return 0;
}
