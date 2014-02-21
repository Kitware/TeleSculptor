/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief OCV image_container implementation
 */

#include "image_container.h"
#include "mat_image_memory.h"

namespace maptk
{

namespace ocv
{


/// Constructor - convert base image container to cv::Mat
image_container
::image_container(const maptk::image_container& image_cont)
{
  // testing if image_cont is an ocv image container
  const ocv::image_container* oic =
      dynamic_cast<const ocv::image_container*>(&image_cont);
  if( oic )
  {
    this->data_ = oic->data_;
  }
  else
  {
    this->data_ = maptk_to_ocv(image_cont.get_image());
  }
}


/// The size of the image data in bytes
/// This size includes all allocated image memory,
/// which could be larger than width*height*depth.
size_t
image_container
::size() const
{
  return data_.rows * data_.step;
}


/// Convert an OpenCV cv::Mat to a MAPTK image
image
image_container
::ocv_to_maptk(const cv::Mat& img)
{
  image_memory_sptr memory(new mat_image_memory(img));

  return image(memory, img.data,
               img.cols, img.rows, img.channels(),
               img.elemSize(), img.step, 1);
}


/// Convert a MAPTK image to an OpenCV cv::Mat
cv::Mat
image_container
::maptk_to_ocv(const image& img)
{
  // cv::Mat is limited in the image data layouts that it supports.
  // Color channels must be interleaved (d_step==1) and the
  // step between columns must equal the number of channels (w_step==depth).
  // If the image does not have these properties we must allocate
  // a new cv::Mat and deep copy the data.  Otherwise, share memory.
  if( img.d_step() == 1 &&
      img.w_step() == static_cast<ptrdiff_t>(img.depth()) )
  {
    image_memory_sptr memory = img.memory();
    cv::Mat out(static_cast<int>(img.height()), static_cast<int>(img.width()),
                CV_MAKETYPE(CV_8U, static_cast<int>(img.depth())),
                memory->data(), img.h_step());

    // if this MAPTK image is already wrapping cv::Mat allocated data,
    // then restore the original cv::Mat reference counter.
    if( mat_image_memory* mat_memory =
          dynamic_cast<mat_image_memory*>(memory.get()) )
    {
      // extract the existing reference counter from the MAPTK wrapper
      out.refcount = mat_memory->get_ref_counter();
      out.addref();
    }
    // TODO use MatAllocator to share memory with image_memory
    return out;
  }

  // allocated a new cv::Mat
  cv::Mat out(static_cast<int>(img.height()), static_cast<int>(img.width()),
              CV_MAKETYPE(CV_8U, static_cast<int>(img.depth())));
  // wrap the new image as a MAPTK image (always a shallow copy)
  image new_img = ocv_to_maptk(out);
  new_img.copy_from(img);

  return out;
}


/// Extract a cv::Mat from any image container
cv::Mat
image_container_to_ocv_matrix(const maptk::image_container& img)
{
  if( const ocv::image_container* c =
          dynamic_cast<const ocv::image_container*>(&img) )
  {
    return c->get_Mat();
  }
  return ocv::image_container::maptk_to_ocv(img.get_image());
}


} // end namespace ocv

} // end namespace maptk
