/*ckwg +29
 * Copyright 2014 by Kitware, Inc.
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

#include "image_container.h"

#include <vil/vil_image_view.h>
#include <vil/vil_save.h>

#include <viscl/core/manager.h>
#include <viscl/tasks/gaussian_smooth.h>
#include <viscl/vxl/transfer.h>


namespace maptk
{

namespace vcl
{


/// Constructor - convert base image container to a VisCL image
viscl_image_container
::viscl_image_container(const image_container& image_cont)
: data_(image_container_to_viscl(image_cont))
{
}


/// The size of the image data in bytes
size_t
viscl_image_container
::size() const
{
  return data_.mem_size();
}


/// Convert a VisCL image to a MAPTK image
image
viscl_image_container
::viscl_to_maptk(const viscl::image& img_cl)
{
  size_t width = img_cl.width();
  size_t height = img_cl.height();
  image img(width, height);

  cl::size_t<3> origin;
  origin.push_back(0);
  origin.push_back(0);
  origin.push_back(0);

  cl::size_t<3> region;
  region.push_back(width);
  region.push_back(height);
  region.push_back(1);

  viscl::cl_queue_t q = viscl::manager::inst()->create_queue();
  q->enqueueReadImage(*img_cl().get(), CL_TRUE, origin, region,
                      0, 0, img.memory()->data());

  return img;
}


/// Convert a MAPTK image to a VisCL image
viscl::image
viscl_image_container
::maptk_to_viscl(const image& img)
{
  cl::ImageFormat img_fmt;
  img_fmt = cl::ImageFormat(CL_INTENSITY, CL_UNORM_INT8);

  // viscl::image is only able to display single channel images at the moment
  // it also only supports byte and float images below only byte are supported
  if( img.depth() == 1 )
  {
    return viscl::image(boost::make_shared<cl::Image2D>(
                          viscl::manager::inst()->get_context(),
                          CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR,
                          img_fmt,
                          img.width(),
                          img.height(),
                          0,
                          (void *)img.first_pixel()));
  }

  if (img.depth() == 3)
  {
    //Convert color image to a grey scale image and upload it.
    unsigned char *grey = new unsigned char [img.width() * img.height()];
    for (unsigned int j = 0; j < img.height(); j++)
    {
      for (unsigned int i = 0; i < img.width(); i++)
      {
        double value = 0.2125 * img(i,j,0) + 0.7154 * img(i,j,1) + 0.0721 * img(i,j,2);
        grey[j * img.width() + i] = static_cast<unsigned char>(value);
      }
    }

    viscl::image image = viscl::image(boost::make_shared<cl::Image2D>(
                                        viscl::manager::inst()->get_context(),
                                        CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR,
                                        img_fmt,
                                        img.width(),
                                        img.height(),
                                        0,
                                        grey));

    delete [] grey;

    return image;
  }

  //TODO: Throw exception
  return viscl::image();
}


/// Extract a VisCL image from any image container
viscl::image
image_container_to_viscl(const image_container& img)
{
  if( const viscl_image_container* c =
          dynamic_cast<const viscl_image_container*>(&img) )
  {
    return c->get_viscl_image();
  }
  return viscl_image_container::maptk_to_viscl(img.get_image());
}


} // end namespace vcl

} // end namespace maptk
