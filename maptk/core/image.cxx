/*ckwg +5
 * Copyright 2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include "image.h"


namespace maptk
{

simple_image
::~simple_image()
{
  delete [] data_;
}


/// Constructor that allocates image memory
simple_image
::simple_image(size_t width, size_t height, size_t depth, bool interleave)
: size_(width*height*depth),
  data_(new byte[size_]),
  first_pixel_(data_),
  width_(width),
  height_(height),
  depth_(depth),
  w_step_(1),
  h_step_(width),
  d_step_(width*height)
{
  if( interleave )
  {
    d_step_ = 1;
    w_step_ = depth;
    h_step_ = depth*width;
  }
}


/// Constructor that points at existing memory
simple_image
::simple_image(const byte* first_pixel, size_t width, size_t height, size_t depth,
               ptrdiff_t w_step, ptrdiff_t h_step, ptrdiff_t d_step)
: size_(0),
  data_(NULL),
  first_pixel_(const_cast<byte*>(first_pixel)),
  width_(width),
  height_(height),
  depth_(depth),
  w_step_(w_step),
  h_step_(h_step),
  d_step_(d_step)
{
}

} // end namespace maptk
