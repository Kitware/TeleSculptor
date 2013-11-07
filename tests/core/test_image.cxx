/*ckwg +5
 * Copyright 2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
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
  if( img1.width()  != img2.width()  ||
      img1.height() != img2.height() ||
      img1.depth()  != img2.depth()  )
  {
    TEST_ERROR("images are not the same size");
  }
  for( unsigned k=0; k<img1.depth(); ++k)
  {
    for( unsigned j=0; j<img1.height(); ++j)
    {
      for( unsigned i=0; i<img1.width(); ++i)
      {
        if( img1(i,j,k) != img2(i,j,k) )
        {
          std::cerr << "image differ at "<< i<<", "<<j<<", "<<k<<std::endl;
        }
      }
    }
  }
  if( ! equal_content(img1, img2) )
  {
    TEST_ERROR("Images should have equal content but do not");
  }
}
