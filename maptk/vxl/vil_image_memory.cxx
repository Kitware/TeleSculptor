/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief VXL image memory implementation
 */

#include "vil_image_memory.h"


namespace maptk
{

namespace vxl
{


/// Create space for n bytes
void
maptk_memory_chunk
::set_size(unsigned long n, vil_pixel_format pixel_format)
{
  if( n != size_ )
  {
    maptk_data_ = image_memory_sptr(new image_memory(n));
    size_ = n;
  }
  pixel_format_ = pixel_format;
}


} // end namespace vxl

} // end namespace maptk
