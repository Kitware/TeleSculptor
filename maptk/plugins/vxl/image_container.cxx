/*ckwg +29
 * Copyright 2013-2014 by Kitware, Inc.
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
 * \brief VXL image container implementation
 */

#include "image_container.h"

#include <maptk/plugins/vxl/vil_image_memory.h>


namespace maptk
{

namespace vxl
{


/// Constructor - convert base image container to vil
image_container
::image_container(const maptk::image_container& image_cont)
{
  const vxl::image_container* vic =
      dynamic_cast<const vxl::image_container*>(&image_cont);
  if( vic )
  {
    this->data_ = vic->data_;
  }
  else
  {
    this->data_ = maptk_to_vxl(image_cont.get_image());
  }
}


/// The size of the image data in bytes
/**
 * This size includes all allocated image memory,
 * which could be larger than width*height*depth.
 */
size_t
image_container
::size() const
{
  if( !data_ )
  {
    return 0;
  }
  return data_.memory_chunk()->size();
}


/// Convert a VXL vil_image_view to a MAPTK image
image
image_container
::vxl_to_maptk(const vil_image_view<vxl_byte>& img)
{
  vil_memory_chunk_sptr chunk = img.memory_chunk();
  image_memory_sptr memory;

  // prevent nested wrappers when converting back and forth.
  // if this vil_image_view is already wrapping MAPTK data,
  // then extract the underlying MAPTK data instead of wrapping
  if( maptk_memory_chunk* maptk_chunk =
        dynamic_cast<maptk_memory_chunk*>(chunk.ptr()) )
  {
    // extract the existing MAPTK memory from the vil wrapper
    memory = maptk_chunk->memory();
  }
  else
  {
    // create a MAPTK wrapper around the vil memory chunk
    memory = image_memory_sptr(new vil_image_memory(chunk));
  }

  return image(memory, img.top_left_ptr(),
               img.ni(), img.nj(), img.nplanes(),
               img.istep(), img.jstep(), img.planestep());
}


/// Convert a MAPTK image to a VXL vil_image_view
vil_image_view<vxl_byte>
image_container
::maptk_to_vxl(const image& img)
{
  image_memory_sptr memory = img.memory();
  vil_memory_chunk_sptr chunk;

  // prevent nested wrappers when converting back and forth.
  // if this MAPTK image is already wrapping vil data,
  // then extract the underlying vil data instead of wrapping
  if( vil_image_memory* vil_memory =
        dynamic_cast<vil_image_memory*>(memory.get()) )
  {
    // extract the existing vil_memory_chunk from the MAPTK wrapper
    chunk = vil_memory->memory_chunk();
  }
  else
  {
    // create a vil wrapper around the MAPTK memory
    chunk = new maptk_memory_chunk(memory);
  }

  return vil_image_view<vxl_byte>(chunk, img.first_pixel(),
                                  static_cast<unsigned int>(img.width()),
                                  static_cast<unsigned int>(img.height()),
                                  static_cast<unsigned int>(img.depth()),
                                  img.w_step(), img.h_step(), img.d_step());
}


} // end namespace vxl

} // end namespace maptk
