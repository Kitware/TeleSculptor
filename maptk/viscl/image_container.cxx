/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include "image_container.h"

#include <viscl/core/manager.h>

namespace maptk
{

namespace vcl
{


/// Constructor - convert base image container to a VisCL image
viscl_image_container
::viscl_image_container(const image_container& image_cont)
{
  const viscl_image_container* vic =
      dynamic_cast<const viscl_image_container*>(&image_cont);
  if( vic )
  {
    this->data_ = vic->data_;
  }
  else
  {
    this->data_ = maptk_to_viscl(image_cont.get_image());
  }
}


/// The size of the image data in bytes
/// This size includes all allocated image memory,
/// which could be larger than width*height*depth.
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
  // viscl::image is only able to display single channel images at the moment
  // it also only supports byte and float images below only byte are supported
  if( img.depth() == 1 )
  {
    image_memory_sptr memory = img.memory();
    cl::ImageFormat img_fmt;
    img_fmt = cl::ImageFormat(CL_INTENSITY, CL_UNORM_INT8);

    return viscl::image(boost::make_shared<cl::Image2D>(
              viscl::manager::inst()->get_context(),
              CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR,
              img_fmt,
              img.width(),
              img.height(),
              0,
              (void *)img.first_pixel()));
  }

  //TODO: throw exception for color image, or return image with CL_RGB
  //image but no viscl algorithm could use it
  return viscl::image();
}


// Extract a VisCL image from any image container
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


} // end namespace viscl

} // end namespace maptk
