/*ckwg +29
 * Copyright 2013-2015 by Kitware, Inc.
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
 * \brief core image class implementation
 */

#include "image.h"
#include <cstring>


namespace maptk
{

/// Default Constructor
image_memory
::image_memory()
: data_(0),
  size_(0)
{
}


/// Constructor - allocated n bytes
image_memory
::image_memory(size_t n)
: data_(new char[n]),
  size_(n)
{
}


/// Copy Constructor
image_memory
::image_memory(const image_memory& other)
: data_(new char[other.size()]),
  size_(other.size())
{
  std::memcpy(data_, other.data_, size_);
}


/// Destructor
image_memory
::~image_memory()
{
  delete [] reinterpret_cast<char*>(data_);
}


/// Assignment operator
image_memory&
image_memory
::operator=(const image_memory& other)
{
  if( this == &other )
  {
    return *this;
  }
  if( size_ != other.size_ )
  {
    delete [] reinterpret_cast<char*>(data_);
    data_ = 0;
    if (other.size_ > 0)
    {
      data_ = new char[other.size_];
    }
    size_ = other.size_;
  }
  std::memcpy(data_, other.data_, size_);
  return *this;
}


/// Return a pointer to the allocated memory
void*
image_memory
::data()
{
  return data_;
}


//======================================================================


/// Default Constructor
image
::image()
: data_(),
  first_pixel_(NULL),
  width_(0),
  height_(0),
  depth_(0),
  w_step_(0),
  h_step_(0),
  d_step_(0)
{
}


/// Constructor that allocates image memory
image
::image(size_t width, size_t height, size_t depth, bool interleave)
: data_(new image_memory(width*height*depth)),
  first_pixel_(reinterpret_cast<byte*>(data_->data())),
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
image
::image(const byte* first_pixel, size_t width, size_t height, size_t depth,
        ptrdiff_t w_step, ptrdiff_t h_step, ptrdiff_t d_step)
: data_(),
  first_pixel_(const_cast<byte*>(first_pixel)),
  width_(width),
  height_(height),
  depth_(depth),
  w_step_(w_step),
  h_step_(h_step),
  d_step_(d_step)
{
}


/// Constructor that shares memory with another image
image
::image(const image_memory_sptr& mem,
        const byte* first_pixel, size_t width, size_t height, size_t depth,
        ptrdiff_t w_step, ptrdiff_t h_step, ptrdiff_t d_step)
: data_(mem),
  first_pixel_(const_cast<byte*>(first_pixel)),
  width_(width),
  height_(height),
  depth_(depth),
  w_step_(w_step),
  h_step_(h_step),
  d_step_(d_step)
{
}


/// Copy Constructor
image
::image(const image& other)
: data_(other.data_),
  first_pixel_(other.first_pixel_),
  width_(other.width_),
  height_(other.height_),
  depth_(other.depth_),
  w_step_(other.w_step_),
  h_step_(other.h_step_),
  d_step_(other.d_step_)
{
}


/// Assignment operator
const image&
image
::operator=(const image& other)
{
  if( this == &other )
  {
    return *this;
  }
  data_ = other.data_;
  first_pixel_ = other.first_pixel_;
  width_ = other.width_;
  height_ = other.height_;
  depth_ = other.depth_;
  w_step_ = other.w_step_;
  h_step_ = other.h_step_;
  d_step_ = other.d_step_;
  return *this;
}


/// The size of the image data in bytes
size_t
image
::size() const
{
  if( !data_ )
  {
    return 0;
  }
  return data_->size();
}


/// Deep copy the image data from another image into this one
void
image
::copy_from(const image& other)
{
  set_size(other.width_, other.height_, other.depth_);

  // TODO check if the images are contiguous and use a
  // memcpy for more efficient copying

  const ptrdiff_t o_d_step = other.d_step();
  const ptrdiff_t o_h_step = other.h_step();
  const ptrdiff_t o_w_step = other.w_step();

  const byte* o_data = other.first_pixel();
  byte* data = this->first_pixel_;
  for (unsigned int d=0; d<depth_; ++d, o_data+=o_d_step, data+=d_step_)
  {
    const byte* o_row = o_data;
    byte* row = data;
    for (unsigned int h=0; h<height_; ++h, o_row+=o_h_step, row+=h_step_)
    {
      const byte* o_pixel = o_row;
      byte* pixel = row;
      for (unsigned int w=0; w<width_; ++w, o_pixel+=o_w_step, pixel+=w_step_)
      {
        *pixel = *o_pixel;
      }
    }
  }
}


/// Set the size of the image.
void
image
::set_size(size_t width, size_t height, size_t depth)
{
  if( width == width_ && height == height_ && depth == depth_ )
  {
    return;
  }

  data_ = image_memory_sptr(new image_memory(width*height*depth));
  width_ = width;
  height_ = height;
  depth_ = depth;
  first_pixel_ = reinterpret_cast<byte*>(data_->data());

  // preserve the pixel ordering (e.g. interleaved) as much as possible
  if( w_step_ == 0 || w_step_ != static_cast<ptrdiff_t>(depth_) )
  {
    w_step_ = 1;
  }
  h_step_ = width * w_step_;
  d_step_ = (w_step_ == 1) ? width*height : 1;
}


/// Compare to images to see if the pixels have the same values.
bool equal_content(const image& img1, const image& img2)
{
  if( img1.width()  != img2.width()  ||
      img1.height() != img2.height() ||
      img1.depth()  != img2.depth()  )
  {
    return false;
  }
  for( unsigned k=0; k<img1.depth(); ++k)
  {
    for( unsigned j=0; j<img1.height(); ++j)
    {
      for( unsigned i=0; i<img1.width(); ++i)
      {
        if( img1(i,j,k) != img2(i,j,k) )
        {
          return false;
        }
      }
    }
  }
  return true;
}


/// Transform a given image into a second image given a unary function
void transform_image( image const &in, image &out,
                      image::byte (*op)(image::byte const &) )
{
  // The other image must be of the same dimensions as this image.
  assert( in.width() == out.width() );
  assert( in.height() == out.height() );
  assert( in.depth() == out.depth() );

  // determine which order to traverse dimensions
  // [0] -> smalled distance between values
  // [2] -> greatest distance between values
  size_t side_len[3];
  ptrdiff_t step_size[3];
  bool wBh = in.width() < in.height(),
       dBh = in.depth() < in.height(),
       dBw = in.depth() < in.width();
  size_t w_idx = (!wBh) + dBw,
         h_idx = wBh + dBh,
         d_idx = (!dBw) + (!dBh);
  side_len[w_idx] = in.width();
  side_len[h_idx] = in.height();
  side_len[d_idx] = in.depth();
  step_size[w_idx] = in.w_step();
  step_size[h_idx] = in.h_step();
  step_size[d_idx] = in.d_step();

  unsigned _0, _1, _2;
  size_t pix_idx;
  for( _2 = 0; _2 < side_len[2]; ++_2 )
  {
    for( _1 = 0; _1 < side_len[1]; ++_1 )
    {
      for( _0 = 0; _0 < side_len[0]; ++_0 )
      {
        pix_idx = _0*step_size[0] + _1*step_size[1] + _2*step_size[2];
        out.first_pixel()[pix_idx] = op( in.first_pixel()[pix_idx] );
      }
    }
  }
}


} // end namespace maptk
