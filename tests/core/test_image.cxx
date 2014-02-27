/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief core image class tests
 */

#include <test_common.h>

#include <maptk/core/image.h>

#define TEST_ARGS ()

DECLARE_TEST_MAP();

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
