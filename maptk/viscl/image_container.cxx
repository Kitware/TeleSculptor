/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include "image_container.h"

namespace maptk
{

namespace viscl
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
  return 0; // TODO return image size in bytes
}


/// Convert a VisCL image to a MAPTK image
//image
//viscl_image_container
//::viscl_to_maptk(const type& img)
//{
//  // TODO download image data
//  return image(memory, img.data,
//               img.cols, img.rows, img.channels(),
//               img.elemSize(), img.step, 1);
//}


/// Convert a MAPTK image to a VisCL image
//type
//viscl_image_container
//::maptk_to_viscl(const image& img)
//{
//  //TODO upload image to GPU
//  return data;
//}


/// Extract a VisCL image from any image container
// TODO define 'type'
//type
//image_container_to_viscl(const image_container& img)
//{
//  if( const viscl_image_container* c =
//          dynamic_cast<const viscl_image_container*>(&img) )
//  {
//    return c->get_viscl_image();
//  }
//  return viscl_image_container::maptk_to_viscl(img.get_image());
//}


} // end namespace viscl

} // end namespace maptk
