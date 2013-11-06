/*ckwg +5
 * Copyright 2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include <test_common.h>

#include <maptk/vxl/image_container.h>
#include <maptk/vxl/image_io.h>

#define TEST_ARGS ()

DECLARE_TEST_MAP();

int
main(int argc, char* argv[])
{
  CHECK_ARGS(1);

  testname_t const testname = argv[1];

  RUN_TEST(testname);
}


IMPLEMENT_TEST(image_convert)
{
  using namespace maptk;
  maptk::image img(200,300,3);
  for( unsigned int p=0; p<img.depth(); ++p )
  {
    for( unsigned int j=0; j<img.height(); ++j )
    {
      for( unsigned int i=0; i<img.width(); ++i )
      {
        img(i,j,p) = ((i/(5*(p+1)))%2) * 100 + ((j/(5*(p+1)))%2) * 100;
      }
    }
  }
  image_container_sptr c(new simple_image_container(img));
  vxl::image_io io;
  io.save("test.png", c);
  image_container_sptr c2 = io.load("test.png");
  maptk::image img2 = c2->get_image();
  if( ! equal_content(img, img2) )
  {
    TEST_ERROR("Saved image is not identical to loaded image");
  }
}
