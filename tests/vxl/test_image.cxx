/*ckwg +5
 * Copyright 2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include <test_common.h>

#include <cstdio>
#include <maptk/vxl/register.h>
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


IMPLEMENT_TEST(factory)
{
  using namespace maptk;
  vxl::register_algorithms();
  typedef boost::shared_ptr<algo::image_io> image_io_sptr;
  image_io_sptr img_io = maptk::algo::image_io::create("vxl");
  if (!img_io)
  {
    TEST_ERROR("Unable to create image_io algorithm of type vxl");
  }
  if (typeid(*img_io.get()) != typeid(vxl::image_io))
  {
    TEST_ERROR("Factory method did not construct the correct type");
  }
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
  if( std::remove("test.png") != 0 )
  {
    TEST_ERROR("Unable to delete temporary image file.");
  }
}
