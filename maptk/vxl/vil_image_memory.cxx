/*ckwg +5
 * Copyright 2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include "vil_image_memory.h"


namespace maptk
{

namespace vxl
{



/// Reallocate memory for n bytes
/// Do nothing if size has not changed
void
vil_image_memory
::set_size(size_t n)
{
  vil_data_->set_size(n, VIL_PIXEL_FORMAT_BYTE);
  size_ = n;
}


/// Create space for n bytes
void
maptk_memory_chunk
::set_size(unsigned long n, vil_pixel_format pixel_format)
{
  maptk_data_->set_size(n);
  size_ = n;
}



} // end namespace vxl

} // end namespace maptk
