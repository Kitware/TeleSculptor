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
 * \brief core image class tests
 */

#include <test_common.h>

#include <maptk/image.h>

#define TEST_ARGS ()

DECLARE_TEST_MAP();


namespace // anonymous
{
  // Helper methods for tests in this file

  // For use in the transform_image function

  static maptk::image::byte val_zero_op( maptk::image::byte const & /*b*/ )
  {
    return 0;
  }

  static maptk::image::byte val_incr_op_i = 0;
  static maptk::image::byte val_incr_op( maptk::image::byte const &b )
  {
    return val_incr_op_i++;
  }

}


int
main(int argc, char* argv[])
{
  CHECK_ARGS(1);

  testname_t const testname = argv[1];

  RUN_TEST(testname);
}


IMPLEMENT_TEST(default_constructor)
{
  maptk::image img;
  if (img.size() != 0        ||
      img.first_pixel() != 0 ||
      img.width() != 0       ||
      img.height() != 0      ||
      img.depth() != 0       )
  {
    TEST_ERROR("The default image is not empty");
  }
}


IMPLEMENT_TEST(constructor)
{
  {
    maptk::image img(200,300);
    if (img.width() != 200  ||
        img.height() != 300 ||
        img.depth() != 1 )
    {
      TEST_ERROR("Constructed image does not have correct dimensions");
    }

    if (img.size() != 200*300)
    {
      TEST_ERROR("Constructor did not allocate correct amount of memory");
    }
  }
  {
    maptk::image img(200,300,3);
    if (img.width() != 200  ||
        img.height() != 300 ||
        img.depth() != 3 )
    {
      TEST_ERROR("Constructed image does not have correct dimensions");
    }

    if (img.size() != 200*300*3)
    {
      TEST_ERROR("Constructor did not allocate correct amount of memory");
    }
  }
}


IMPLEMENT_TEST(copy_constructor)
{
  maptk::image img(100,75,2);
  maptk::image img_cpy(img);
  if (img.width()  != img_cpy.width()  ||
      img.height() != img_cpy.height() ||
      img.depth()  != img_cpy.depth()  ||
      img.w_step() != img_cpy.w_step() ||
      img.h_step() != img_cpy.h_step() ||
      img.d_step() != img_cpy.d_step() ||
      img.first_pixel() != img_cpy.first_pixel() ||
      img.memory() != img_cpy.memory() )
  {
    TEST_ERROR("Copy constructed image does not match original");
  }
}


IMPLEMENT_TEST(set_size)
{
  maptk::image img(10, 20, 4);
  unsigned char* data = img.first_pixel();
  img.set_size(10, 20, 4);
  if( img.first_pixel() != data )
  {
    TEST_ERROR("Calling set_size with the existing size should keep the same memory");
  }
  // keep another copy of the original image to prevent the original memory from
  // being deleted and then reallocated.
  maptk::image img_copy = img;
  img.set_size(20, 10, 4);
  if( img.first_pixel() == data )
  {
    TEST_ERROR("Calling set_size with a new size should allocate new memory");
  }
  if( img.width() != 20 || img.height() != 10 || img.depth() != 4 )
  {
    TEST_ERROR("Image does not have correct size after set_size");
  }
}


IMPLEMENT_TEST(copy_from)
{
  unsigned w=100, h=200, d=3;
  maptk::image img1(w,h,d);
  for(unsigned k=0; k<d; ++k)
  {
    for(unsigned j=0; j<h; ++j)
    {
      for(unsigned i=0; i<w; ++i)
      {
        img1(i,j,k)  = static_cast<unsigned char>((w*h*k + w*j + i) % 255);
      }
    }
  }

  maptk::image img2;
  img2.copy_from(img1);
  if( img1.first_pixel() == img2.first_pixel() )
  {
    TEST_ERROR("Deep copied images should not share the same memory");
  }
  if( ! equal_content(img1, img2) )
  {
    TEST_ERROR("Deep copied images should have the same content");
  }

  maptk::image img3(200, 400, 3);
  // create a view into the center of img3
  maptk::image img4(img3.memory(), img3.first_pixel()+50*200 + 50,
                    w, h, d,
                    1, 200, 200*400);
  // copy data into the view
  unsigned char * data = img4.first_pixel();
  img4.copy_from(img1);
  if( img4.first_pixel() != data )
  {
    TEST_ERROR("Deep copying with the correct size should not reallocate memory");
  }
  if( ! equal_content(img1, img4) )
  {
    TEST_ERROR("Deep copied images (in a view) should have the same content");
  }
}


IMPLEMENT_TEST(equal_content)
{
  unsigned w=100, h=200, d=3;
  maptk::image img1(w,h,d), img2(w,h,d,true);
  if(img1.memory() == img2.memory())
  {
    TEST_ERROR("Test images should not have the same memory");
  }
  if(img1.w_step() == img2.w_step())
  {
    TEST_ERROR("Test images should not have the same memory layout");
  }
  for(unsigned k=0; k<d; ++k)
  {
    for(unsigned j=0; j<h; ++j)
    {
      for(unsigned i=0; i<w; ++i)
      {
        img1(i,j,k) = img2(i,j,k) = static_cast<unsigned char>((w*h*k + w*j + i) % 255);
      }
    }
  }
  if( ! equal_content(img1, img2) )
  {
    TEST_ERROR("Images should have equal content but do not");
  }
}


IMPLEMENT_TEST(transform_image)
{
  // Testing that the transform image traverses pixels in memory order
  unsigned w=3, h=3, d=3;
  maptk::image img;

  // an image with traditional stepping ( w < h < d )
  {
    img = maptk::image( w, h, d , false );

    // Zeroing image data
    maptk::transform_image( img, val_zero_op );
    TEST_EQUAL( "normal-zero (0,0,0)", (unsigned)img(0,0,0), 0 );
    TEST_EQUAL( "normal-zero (1,0,0)", (unsigned)img(1,0,0), 0 );
    TEST_EQUAL( "normal-zero (2,0,0)", (unsigned)img(2,0,0), 0 );
    TEST_EQUAL( "normal-zero (0,1,0)", (unsigned)img(0,1,0), 0 );
    TEST_EQUAL( "normal-zero (1,1,0)", (unsigned)img(1,1,0), 0 );
    TEST_EQUAL( "normal-zero (2,1,0)", (unsigned)img(2,1,0), 0 );
    TEST_EQUAL( "normal-zero (0,2,0)", (unsigned)img(0,2,0), 0 );
    TEST_EQUAL( "normal-zero (1,2,0)", (unsigned)img(1,2,0), 0 );
    TEST_EQUAL( "normal-zero (2,2,0)", (unsigned)img(2,2,0), 0 );
    TEST_EQUAL( "normal-zero (0,0,1)", (unsigned)img(0,0,1), 0 );
    TEST_EQUAL( "normal-zero (1,0,1)", (unsigned)img(1,0,1), 0 );
    TEST_EQUAL( "normal-zero (2,0,1)", (unsigned)img(2,0,1), 0 );
    TEST_EQUAL( "normal-zero (0,1,1)", (unsigned)img(0,1,1), 0 );
    TEST_EQUAL( "normal-zero (1,1,1)", (unsigned)img(1,1,1), 0 );
    TEST_EQUAL( "normal-zero (2,1,1)", (unsigned)img(2,1,1), 0 );
    TEST_EQUAL( "normal-zero (0,2,1)", (unsigned)img(0,2,1), 0 );
    TEST_EQUAL( "normal-zero (1,2,1)", (unsigned)img(1,2,1), 0 );
    TEST_EQUAL( "normal-zero (2,2,1)", (unsigned)img(2,2,1), 0 );
    TEST_EQUAL( "normal-zero (0,0,2)", (unsigned)img(0,0,2), 0 );
    TEST_EQUAL( "normal-zero (1,0,2)", (unsigned)img(1,0,2), 0 );
    TEST_EQUAL( "normal-zero (2,0,2)", (unsigned)img(2,0,2), 0 );
    TEST_EQUAL( "normal-zero (0,1,2)", (unsigned)img(0,1,2), 0 );
    TEST_EQUAL( "normal-zero (1,1,2)", (unsigned)img(1,1,2), 0 );
    TEST_EQUAL( "normal-zero (2,1,2)", (unsigned)img(2,1,2), 0 );
    TEST_EQUAL( "normal-zero (0,2,2)", (unsigned)img(0,2,2), 0 );
    TEST_EQUAL( "normal-zero (1,2,2)", (unsigned)img(1,2,2), 0 );
    TEST_EQUAL( "normal-zero (2,2,2)", (unsigned)img(2,2,2), 0 );

    // Assinging value
    val_incr_op_i = 0;
    maptk::transform_image( img, val_incr_op );
    TEST_EQUAL( "normal (0,0,0)", (unsigned)img(0,0,0), 0 );
    TEST_EQUAL( "normal (1,0,0)", (unsigned)img(1,0,0), 1 );
    TEST_EQUAL( "normal (2,0,0)", (unsigned)img(2,0,0), 2 );
    TEST_EQUAL( "normal (0,1,0)", (unsigned)img(0,1,0), 3 );
    TEST_EQUAL( "normal (1,1,0)", (unsigned)img(1,1,0), 4 );
    TEST_EQUAL( "normal (2,1,0)", (unsigned)img(2,1,0), 5 );
    TEST_EQUAL( "normal (0,2,0)", (unsigned)img(0,2,0), 6 );
    TEST_EQUAL( "normal (1,2,0)", (unsigned)img(1,2,0), 7 );
    TEST_EQUAL( "normal (2,2,0)", (unsigned)img(2,2,0), 8 );
    TEST_EQUAL( "normal (0,0,1)", (unsigned)img(0,0,1), 9 );
    TEST_EQUAL( "normal (1,0,1)", (unsigned)img(1,0,1), 10 );
    TEST_EQUAL( "normal (2,0,1)", (unsigned)img(2,0,1), 11 );
    TEST_EQUAL( "normal (0,1,1)", (unsigned)img(0,1,1), 12 );
    TEST_EQUAL( "normal (1,1,1)", (unsigned)img(1,1,1), 13 );
    TEST_EQUAL( "normal (2,1,1)", (unsigned)img(2,1,1), 14 );
    TEST_EQUAL( "normal (0,2,1)", (unsigned)img(0,2,1), 15 );
    TEST_EQUAL( "normal (1,2,1)", (unsigned)img(1,2,1), 16 );
    TEST_EQUAL( "normal (2,2,1)", (unsigned)img(2,2,1), 17 );
    TEST_EQUAL( "normal (0,0,2)", (unsigned)img(0,0,2), 18 );
    TEST_EQUAL( "normal (1,0,2)", (unsigned)img(1,0,2), 19 );
    TEST_EQUAL( "normal (2,0,2)", (unsigned)img(2,0,2), 20 );
    TEST_EQUAL( "normal (0,1,2)", (unsigned)img(0,1,2), 21 );
    TEST_EQUAL( "normal (1,1,2)", (unsigned)img(1,1,2), 22 );
    TEST_EQUAL( "normal (2,1,2)", (unsigned)img(2,1,2), 23 );
    TEST_EQUAL( "normal (0,2,2)", (unsigned)img(0,2,2), 24 );
    TEST_EQUAL( "normal (1,2,2)", (unsigned)img(1,2,2), 25 );
    TEST_EQUAL( "normal (2,2,2)", (unsigned)img(2,2,2), 26 );
  }

  // an interleaved image ( d < w < h )
  {
    img = maptk::image( w, h, d, true );

    maptk::transform_image( img, val_zero_op );
    TEST_EQUAL( "interleaved-zero (0,0,0)", (unsigned)img(0,0,0), 0 );
    TEST_EQUAL( "interleaved-zero (0,0,1)", (unsigned)img(0,0,1), 0 );
    TEST_EQUAL( "interleaved-zero (0,0,2)", (unsigned)img(0,0,2), 0 );
    TEST_EQUAL( "interleaved-zero (1,0,0)", (unsigned)img(1,0,0), 0 );
    TEST_EQUAL( "interleaved-zero (1,0,1)", (unsigned)img(1,0,1), 0 );
    TEST_EQUAL( "interleaved-zero (1,0,2)", (unsigned)img(1,0,2), 0 );
    TEST_EQUAL( "interleaved-zero (2,0,0)", (unsigned)img(2,0,0), 0 );
    TEST_EQUAL( "interleaved-zero (2,0,1)", (unsigned)img(2,0,1), 0 );
    TEST_EQUAL( "interleaved-zero (2,0,2)", (unsigned)img(2,0,2), 0 );
    TEST_EQUAL( "interleaved-zero (0,1,0)", (unsigned)img(0,1,0), 0 );
    TEST_EQUAL( "interleaved-zero (0,1,1)", (unsigned)img(0,1,1), 0 );
    TEST_EQUAL( "interleaved-zero (0,1,2)", (unsigned)img(0,1,2), 0 );
    TEST_EQUAL( "interleaved-zero (1,1,0)", (unsigned)img(1,1,0), 0 );
    TEST_EQUAL( "interleaved-zero (1,1,1)", (unsigned)img(1,1,1), 0 );
    TEST_EQUAL( "interleaved-zero (1,1,2)", (unsigned)img(1,1,2), 0 );
    TEST_EQUAL( "interleaved-zero (2,1,0)", (unsigned)img(2,1,0), 0 );
    TEST_EQUAL( "interleaved-zero (2,1,1)", (unsigned)img(2,1,1), 0 );
    TEST_EQUAL( "interleaved-zero (2,1,2)", (unsigned)img(2,1,2), 0 );
    TEST_EQUAL( "interleaved-zero (0,2,0)", (unsigned)img(0,2,0), 0 );
    TEST_EQUAL( "interleaved-zero (0,2,1)", (unsigned)img(0,2,1), 0 );
    TEST_EQUAL( "interleaved-zero (0,2,2)", (unsigned)img(0,2,2), 0 );
    TEST_EQUAL( "interleaved-zero (1,2,0)", (unsigned)img(1,2,0), 0 );
    TEST_EQUAL( "interleaved-zero (1,2,1)", (unsigned)img(1,2,1), 0 );
    TEST_EQUAL( "interleaved-zero (1,2,2)", (unsigned)img(1,2,2), 0 );
    TEST_EQUAL( "interleaved-zero (2,2,0)", (unsigned)img(2,2,0), 0 );
    TEST_EQUAL( "interleaved-zero (2,2,1)", (unsigned)img(2,2,1), 0 );
    TEST_EQUAL( "interleaved-zero (2,2,2)", (unsigned)img(2,2,2), 0 );

    val_incr_op_i = 0;
    maptk::transform_image( img, val_incr_op );
    TEST_EQUAL( "interleaved (0,0,0)", (unsigned)img(0,0,0), 0 );
    TEST_EQUAL( "interleaved (0,0,1)", (unsigned)img(0,0,1), 1 );
    TEST_EQUAL( "interleaved (0,0,2)", (unsigned)img(0,0,2), 2 );
    TEST_EQUAL( "interleaved (1,0,0)", (unsigned)img(1,0,0), 3 );
    TEST_EQUAL( "interleaved (1,0,1)", (unsigned)img(1,0,1), 4 );
    TEST_EQUAL( "interleaved (1,0,2)", (unsigned)img(1,0,2), 5 );
    TEST_EQUAL( "interleaved (2,0,0)", (unsigned)img(2,0,0), 6 );
    TEST_EQUAL( "interleaved (2,0,1)", (unsigned)img(2,0,1), 7 );
    TEST_EQUAL( "interleaved (2,0,2)", (unsigned)img(2,0,2), 8 );
    TEST_EQUAL( "interleaved (0,1,0)", (unsigned)img(0,1,0), 9 );
    TEST_EQUAL( "interleaved (0,1,1)", (unsigned)img(0,1,1), 10 );
    TEST_EQUAL( "interleaved (0,1,2)", (unsigned)img(0,1,2), 11 );
    TEST_EQUAL( "interleaved (1,1,0)", (unsigned)img(1,1,0), 12 );
    TEST_EQUAL( "interleaved (1,1,1)", (unsigned)img(1,1,1), 13 );
    TEST_EQUAL( "interleaved (1,1,2)", (unsigned)img(1,1,2), 14 );
    TEST_EQUAL( "interleaved (2,1,0)", (unsigned)img(2,1,0), 15 );
    TEST_EQUAL( "interleaved (2,1,1)", (unsigned)img(2,1,1), 16 );
    TEST_EQUAL( "interleaved (2,1,2)", (unsigned)img(2,1,2), 17 );
    TEST_EQUAL( "interleaved (0,2,0)", (unsigned)img(0,2,0), 18 );
    TEST_EQUAL( "interleaved (0,2,1)", (unsigned)img(0,2,1), 19 );
    TEST_EQUAL( "interleaved (0,2,2)", (unsigned)img(0,2,2), 20 );
    TEST_EQUAL( "interleaved (1,2,0)", (unsigned)img(1,2,0), 21 );
    TEST_EQUAL( "interleaved (1,2,1)", (unsigned)img(1,2,1), 22 );
    TEST_EQUAL( "interleaved (1,2,2)", (unsigned)img(1,2,2), 23 );
    TEST_EQUAL( "interleaved (2,2,0)", (unsigned)img(2,2,0), 24 );
    TEST_EQUAL( "interleaved (2,2,1)", (unsigned)img(2,2,1), 25 );
    TEST_EQUAL( "interleaved (2,2,2)", (unsigned)img(2,2,2), 26 );
  }

  // do weird format
  {
    ptrdiff_t hStep = 1,
              dStep = h,
              wStep = d*h;
    maptk::image_memory weird_img_mem( wStep * hStep * dStep );
    img = maptk::image( (maptk::image::byte*)weird_img_mem.data(),
                        w, h, d,
                        wStep, hStep, dStep );

    maptk::transform_image( img, val_zero_op );
    TEST_EQUAL( "weird-zero (0,0,0)", (unsigned)img(0,0,0), 0 );
    TEST_EQUAL( "weird-zero (0,1,0)", (unsigned)img(0,1,0), 0 );
    TEST_EQUAL( "weird-zero (0,2,0)", (unsigned)img(0,2,0), 0 );
    TEST_EQUAL( "weird-zero (0,0,1)", (unsigned)img(0,0,1), 0 );
    TEST_EQUAL( "weird-zero (0,1,1)", (unsigned)img(0,1,1), 0 );
    TEST_EQUAL( "weird-zero (0,2,1)", (unsigned)img(0,2,1), 0 );
    TEST_EQUAL( "weird-zero (0,0,2)", (unsigned)img(0,0,2), 0 );
    TEST_EQUAL( "weird-zero (0,1,2)", (unsigned)img(0,1,2), 0 );
    TEST_EQUAL( "weird-zero (0,2,2)", (unsigned)img(0,2,2), 0 );
    TEST_EQUAL( "weird-zero (1,0,0)", (unsigned)img(1,0,0), 0 );
    TEST_EQUAL( "weird-zero (1,1,0)", (unsigned)img(1,1,0), 0 );
    TEST_EQUAL( "weird-zero (1,2,0)", (unsigned)img(1,2,0), 0 );
    TEST_EQUAL( "weird-zero (1,0,1)", (unsigned)img(1,0,1), 0 );
    TEST_EQUAL( "weird-zero (1,1,1)", (unsigned)img(1,1,1), 0 );
    TEST_EQUAL( "weird-zero (1,2,1)", (unsigned)img(1,2,1), 0 );
    TEST_EQUAL( "weird-zero (1,0,2)", (unsigned)img(1,0,2), 0 );
    TEST_EQUAL( "weird-zero (1,1,2)", (unsigned)img(1,1,2), 0 );
    TEST_EQUAL( "weird-zero (1,2,2)", (unsigned)img(1,2,2), 0 );
    TEST_EQUAL( "weird-zero (2,0,0)", (unsigned)img(2,0,0), 0 );
    TEST_EQUAL( "weird-zero (2,1,0)", (unsigned)img(2,1,0), 0 );
    TEST_EQUAL( "weird-zero (2,2,0)", (unsigned)img(2,2,0), 0 );
    TEST_EQUAL( "weird-zero (2,0,1)", (unsigned)img(2,0,1), 0 );
    TEST_EQUAL( "weird-zero (2,1,1)", (unsigned)img(2,1,1), 0 );
    TEST_EQUAL( "weird-zero (2,2,1)", (unsigned)img(2,2,1), 0 );
    TEST_EQUAL( "weird-zero (2,0,2)", (unsigned)img(2,0,2), 0 );
    TEST_EQUAL( "weird-zero (2,1,2)", (unsigned)img(2,1,2), 0 );
    TEST_EQUAL( "weird-zero (2,2,2)", (unsigned)img(2,2,2), 0 );

    val_incr_op_i = 0;
    maptk::transform_image( img, val_incr_op );
    TEST_EQUAL( "weird (0,0,0)", (unsigned)img(0,0,0), 0 );
    TEST_EQUAL( "weird (0,1,0)", (unsigned)img(0,1,0), 1 );
    TEST_EQUAL( "weird (0,2,0)", (unsigned)img(0,2,0), 2 );
    TEST_EQUAL( "weird (0,0,1)", (unsigned)img(0,0,1), 3 );
    TEST_EQUAL( "weird (0,1,1)", (unsigned)img(0,1,1), 4 );
    TEST_EQUAL( "weird (0,2,1)", (unsigned)img(0,2,1), 5 );
    TEST_EQUAL( "weird (0,0,2)", (unsigned)img(0,0,2), 6 );
    TEST_EQUAL( "weird (0,1,2)", (unsigned)img(0,1,2), 7 );
    TEST_EQUAL( "weird (0,2,2)", (unsigned)img(0,2,2), 8 );
    TEST_EQUAL( "weird (1,0,0)", (unsigned)img(1,0,0), 9 );
    TEST_EQUAL( "weird (1,1,0)", (unsigned)img(1,1,0), 10 );
    TEST_EQUAL( "weird (1,2,0)", (unsigned)img(1,2,0), 11 );
    TEST_EQUAL( "weird (1,0,1)", (unsigned)img(1,0,1), 12 );
    TEST_EQUAL( "weird (1,1,1)", (unsigned)img(1,1,1), 13 );
    TEST_EQUAL( "weird (1,2,1)", (unsigned)img(1,2,1), 14 );
    TEST_EQUAL( "weird (1,0,2)", (unsigned)img(1,0,2), 15 );
    TEST_EQUAL( "weird (1,1,2)", (unsigned)img(1,1,2), 16 );
    TEST_EQUAL( "weird (1,2,2)", (unsigned)img(1,2,2), 17 );
    TEST_EQUAL( "weird (2,0,0)", (unsigned)img(2,0,0), 18 );
    TEST_EQUAL( "weird (2,1,0)", (unsigned)img(2,1,0), 19 );
    TEST_EQUAL( "weird (2,2,0)", (unsigned)img(2,2,0), 20 );
    TEST_EQUAL( "weird (2,0,1)", (unsigned)img(2,0,1), 21 );
    TEST_EQUAL( "weird (2,1,1)", (unsigned)img(2,1,1), 22 );
    TEST_EQUAL( "weird (2,2,1)", (unsigned)img(2,2,1), 23 );
    TEST_EQUAL( "weird (2,0,2)", (unsigned)img(2,0,2), 24 );
    TEST_EQUAL( "weird (2,1,2)", (unsigned)img(2,1,2), 25 );
    TEST_EQUAL( "weird (2,2,2)", (unsigned)img(2,2,2), 26 );
  }

  // do non-contiguous format
  {
    ptrdiff_t wStep = 7,
              hStep = w * wStep + 11,
              dStep = h * hStep * 3;
    maptk::image_memory non_con_img_mem( wStep * hStep * dStep );
    img = maptk::image( (maptk::image::byte*)non_con_img_mem.data(),
                        w, h, d,
                        wStep, hStep, dStep );

    maptk::transform_image( img, val_zero_op );
    TEST_EQUAL( "non-contiguous-zero (0,0,0)", (unsigned)img(0,0,0), 0 );
    TEST_EQUAL( "non-contiguous-zero (1,0,0)", (unsigned)img(1,0,0), 0 );
    TEST_EQUAL( "non-contiguous-zero (2,0,0)", (unsigned)img(2,0,0), 0 );
    TEST_EQUAL( "non-contiguous-zero (0,1,0)", (unsigned)img(0,1,0), 0 );
    TEST_EQUAL( "non-contiguous-zero (1,1,0)", (unsigned)img(1,1,0), 0 );
    TEST_EQUAL( "non-contiguous-zero (2,1,0)", (unsigned)img(2,1,0), 0 );
    TEST_EQUAL( "non-contiguous-zero (0,2,0)", (unsigned)img(0,2,0), 0 );
    TEST_EQUAL( "non-contiguous-zero (1,2,0)", (unsigned)img(1,2,0), 0 );
    TEST_EQUAL( "non-contiguous-zero (2,2,0)", (unsigned)img(2,2,0), 0 );
    TEST_EQUAL( "non-contiguous-zero (0,0,1)", (unsigned)img(0,0,1), 0 );
    TEST_EQUAL( "non-contiguous-zero (1,0,1)", (unsigned)img(1,0,1), 0 );
    TEST_EQUAL( "non-contiguous-zero (2,0,1)", (unsigned)img(2,0,1), 0 );
    TEST_EQUAL( "non-contiguous-zero (0,1,1)", (unsigned)img(0,1,1), 0 );
    TEST_EQUAL( "non-contiguous-zero (1,1,1)", (unsigned)img(1,1,1), 0 );
    TEST_EQUAL( "non-contiguous-zero (2,1,1)", (unsigned)img(2,1,1), 0 );
    TEST_EQUAL( "non-contiguous-zero (0,2,1)", (unsigned)img(0,2,1), 0 );
    TEST_EQUAL( "non-contiguous-zero (1,2,1)", (unsigned)img(1,2,1), 0 );
    TEST_EQUAL( "non-contiguous-zero (2,2,1)", (unsigned)img(2,2,1), 0 );
    TEST_EQUAL( "non-contiguous-zero (0,0,2)", (unsigned)img(0,0,2), 0 );
    TEST_EQUAL( "non-contiguous-zero (1,0,2)", (unsigned)img(1,0,2), 0 );
    TEST_EQUAL( "non-contiguous-zero (2,0,2)", (unsigned)img(2,0,2), 0 );
    TEST_EQUAL( "non-contiguous-zero (0,1,2)", (unsigned)img(0,1,2), 0 );
    TEST_EQUAL( "non-contiguous-zero (1,1,2)", (unsigned)img(1,1,2), 0 );
    TEST_EQUAL( "non-contiguous-zero (2,1,2)", (unsigned)img(2,1,2), 0 );
    TEST_EQUAL( "non-contiguous-zero (0,2,2)", (unsigned)img(0,2,2), 0 );
    TEST_EQUAL( "non-contiguous-zero (1,2,2)", (unsigned)img(1,2,2), 0 );
    TEST_EQUAL( "non-contiguous-zero (2,2,2)", (unsigned)img(2,2,2), 0 );

    val_incr_op_i = 0;
    maptk::transform_image( img, val_incr_op );
    TEST_EQUAL( "non-contiguous (0,0,0)", (unsigned)img(0,0,0), 0 );
    TEST_EQUAL( "non-contiguous (1,0,0)", (unsigned)img(1,0,0), 1 );
    TEST_EQUAL( "non-contiguous (2,0,0)", (unsigned)img(2,0,0), 2 );
    TEST_EQUAL( "non-contiguous (0,1,0)", (unsigned)img(0,1,0), 3 );
    TEST_EQUAL( "non-contiguous (1,1,0)", (unsigned)img(1,1,0), 4 );
    TEST_EQUAL( "non-contiguous (2,1,0)", (unsigned)img(2,1,0), 5 );
    TEST_EQUAL( "non-contiguous (0,2,0)", (unsigned)img(0,2,0), 6 );
    TEST_EQUAL( "non-contiguous (1,2,0)", (unsigned)img(1,2,0), 7 );
    TEST_EQUAL( "non-contiguous (2,2,0)", (unsigned)img(2,2,0), 8 );
    TEST_EQUAL( "non-contiguous (0,0,1)", (unsigned)img(0,0,1), 9 );
    TEST_EQUAL( "non-contiguous (1,0,1)", (unsigned)img(1,0,1), 10 );
    TEST_EQUAL( "non-contiguous (2,0,1)", (unsigned)img(2,0,1), 11 );
    TEST_EQUAL( "non-contiguous (0,1,1)", (unsigned)img(0,1,1), 12 );
    TEST_EQUAL( "non-contiguous (1,1,1)", (unsigned)img(1,1,1), 13 );
    TEST_EQUAL( "non-contiguous (2,1,1)", (unsigned)img(2,1,1), 14 );
    TEST_EQUAL( "non-contiguous (0,2,1)", (unsigned)img(0,2,1), 15 );
    TEST_EQUAL( "non-contiguous (1,2,1)", (unsigned)img(1,2,1), 16 );
    TEST_EQUAL( "non-contiguous (2,2,1)", (unsigned)img(2,2,1), 17 );
    TEST_EQUAL( "non-contiguous (0,0,2)", (unsigned)img(0,0,2), 18 );
    TEST_EQUAL( "non-contiguous (1,0,2)", (unsigned)img(1,0,2), 19 );
    TEST_EQUAL( "non-contiguous (2,0,2)", (unsigned)img(2,0,2), 20 );
    TEST_EQUAL( "non-contiguous (0,1,2)", (unsigned)img(0,1,2), 21 );
    TEST_EQUAL( "non-contiguous (1,1,2)", (unsigned)img(1,1,2), 22 );
    TEST_EQUAL( "non-contiguous (2,1,2)", (unsigned)img(2,1,2), 23 );
    TEST_EQUAL( "non-contiguous (0,2,2)", (unsigned)img(0,2,2), 24 );
    TEST_EQUAL( "non-contiguous (1,2,2)", (unsigned)img(1,2,2), 25 );
    TEST_EQUAL( "non-contiguous (2,2,2)", (unsigned)img(2,2,2), 26 );
  }

}
