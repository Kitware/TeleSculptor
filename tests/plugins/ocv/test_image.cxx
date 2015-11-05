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
 * \brief test OCV image class
 */

#include <test_common.h>

#include <maptk/plugins/ocv/register_algorithms.h>
#include <maptk/plugins/ocv/image_container.h>
#include <maptk/plugins/ocv/image_io.h>

#define TEST_ARGS ()

DECLARE_TEST_MAP();

int
main(int argc, char* argv[])
{
  CHECK_ARGS(1);

  testname_t const testname = argv[1];

  RUN_TEST(testname);
}

using namespace kwiver::vital;

IMPLEMENT_TEST(factory)
{
  using namespace kwiver::maptk;
  ocv::register_algorithms();
  algo::image_io_sptr img_io = kwiver::vital::algo::image_io::create("ocv");
  if (!img_io)
  {
    TEST_ERROR("Unable to create image_io algorithm of type ocv");
  }
  if (typeid(*img_io.get()) != typeid(ocv::image_io))
  {
    TEST_ERROR("Factory method did not construct the correct type");
  }
}


IMPLEMENT_TEST(image_convert)
{
  using namespace kwiver::maptk;
  kwiver::vital::image img(200,300,3);
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
  ocv::image_io io;
  io.save("test.png", c);
  image_container_sptr c2 = io.load("test.png");
  kwiver::vital::image img2 = c2->get_image();
  if( ! equal_content(img, img2) )
  {
    TEST_ERROR("Saved image is not identical to loaded image");
  }
}
