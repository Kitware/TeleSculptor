/*ckwg +5
 * Copyright 2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include "image_container.h"
#include "vil_image_memory.h"

namespace maptk
{

namespace vxl
{


/// Constructor - convert base image container to vil
vxl_image_container
::vxl_image_container(const image_container& image_cont)
{
  const vxl_image_container* vic =
      dynamic_cast<const vxl_image_container*>(&image_cont);
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
/// This size includes all allocated image memory,
/// which could be larger than width*height*depth.
size_t
vxl_image_container
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
vxl_image_container
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
vxl_image_container
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
                                  img.width(), img.height(), img.depth(),
                                  img.w_step(), img.h_step(), img.d_step());
}


} // end namespace vxl

} // end namespace maptk
