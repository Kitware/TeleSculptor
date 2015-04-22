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
 * \brief maptk::image_container C interface implementation
 */

#include "image_container.h"

#include <maptk/image_container.h>

#include <maptk/c/helpers/c_utils.h>
#include <maptk/c/helpers/image_container.h>


namespace maptk_c
{

SharedPointerCache< maptk::image_container, maptk_image_container_t >
  IMGC_SPTR_CACHE( "image_container" );

}


/// Create a new, simple image container around an image
maptk_image_container_t* maptk_image_container_new_simple( maptk_image_t *img )
{
  STANDARD_CATCH(
    "C::image_container::new_simple", 0,

    maptk::image *maptk_img = reinterpret_cast<maptk::image*>(img);
    maptk::image_container_sptr img_sptr( new maptk::simple_image_container( *maptk_img ) );
    maptk_c::IMGC_SPTR_CACHE.store( img_sptr );
    return reinterpret_cast<maptk_image_container_t*>( img_sptr.get() );
  );
  return 0;
}


/// Destroy a maptk_image_container_t instance
void maptk_image_container_destroy( maptk_image_container_t *img_container,
                                    maptk_error_handle_t *eh )
{
  STANDARD_CATCH(
    "C::image_container::destroy", eh,
    maptk_c::IMGC_SPTR_CACHE.erase( img_container );
  );
}


/// Get the size in bytes of an image container
size_t maptk_image_container_size( maptk_image_container_t *img_c )
{
  STANDARD_CATCH(
    "C::image_container::size", 0,
    return maptk_c::IMGC_SPTR_CACHE.get( img_c )->size();
  );
  return 0;
}


/// Get the width of the given image in pixels
size_t maptk_image_container_width( maptk_image_container_t *img_c )
{
  STANDARD_CATCH(
    "C::image_container::width", 0,
    return maptk_c::IMGC_SPTR_CACHE.get( img_c )->width();
  );
  return 0;
}


/// Get the height of the given image in pixels
size_t maptk_image_container_height( maptk_image_container_t *img_c )
{
  STANDARD_CATCH(
    "C::image_container::height", 0,
    return maptk_c::IMGC_SPTR_CACHE.get( img_c )->height();
  );
  return 0;
}


/// Get the depth (number of channels) of the image
size_t maptk_image_container_depth( maptk_image_container_t *img_c )
{
  STANDARD_CATCH(
    "C::image_container::depth", 0,
    return maptk_c::IMGC_SPTR_CACHE.get( img_c )->depth();
  );
  return 0;
}


/// Get the in-memory image class to access data
maptk_image_t* maptk_image_container_get_image( maptk_image_container_t *img_c )
{
  STANDARD_CATCH(
    "C::image_container::get_image", NULL,
    using namespace maptk_c;
    maptk::image_container_sptr ic_sptr( IMGC_SPTR_CACHE.get( img_c ) );
    return reinterpret_cast<maptk_image_t*>(
      new maptk::image( ic_sptr->get_image() )
    );
  );
  return 0;
}
