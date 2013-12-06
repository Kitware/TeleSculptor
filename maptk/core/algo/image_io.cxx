/*ckwg +5
 * Copyright 2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include "image_io.h"
#include <maptk/core/registrar.h>

namespace maptk
{


// Initialize the static instance of the registrar
template<>
registrar<algo::image_io>* registrar<algo::image_io>::instance_ = 0;


namespace algo
{


/// Register instances of this algorithm
bool
image_io
::register_instance(boost::shared_ptr<image_io> inst)
{
  if (!inst)
  {
    return false;
  }
  return registrar<image_io>::register_item(inst->impl_name(), inst);
}


/// Factory method to make an instance of this algorithm by impl_name
boost::shared_ptr<image_io>
image_io
::create(const std::string& impl_name)
{
  boost::shared_ptr<image_io> inst = registrar<image_io>::find(impl_name);
  if (!inst)
  {
    return inst;
  }
  return inst->clone();
}

/// Return a vector of the impl_name of each registered implementation
std::vector<std::string>
image_io
::registered_names()
{
  return registrar<image_io>::registered_names();
}


} // end namespace algo

} // end namespace maptk
