/*ckwg +5
 * Copyright 2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_ALGO_ALGORITHM_TXX_
#define MAPTK_ALGO_ALGORITHM_TXX_

#include "algorithm.h"
#include <maptk/core/registrar.h>

namespace maptk
{

namespace algo
{


/// Register instances of this algorithm
template <typename Self>
bool
algorithm_def<Self>
::register_instance(boost::shared_ptr<Self> inst)
{
  if (!inst)
  {
    return false;
  }
  return registrar<Self>::register_item(inst->impl_name(), inst);
}


/// Factory method to make an instance of this algorithm by impl_name
template <typename Self>
boost::shared_ptr<Self>
algorithm_def<Self>
::create(const std::string& impl_name)
{
  boost::shared_ptr<Self> inst = registrar<Self>::find(impl_name);
  if (!inst)
  {
    return inst;
  }
  return inst->clone();
}


/// Return a vector of the impl_name of each registered implementation
template <typename Self>
std::vector<std::string>
algorithm_def<Self>
::registered_names()
{
  return registrar<Self>::registered_names();
}


} // end namespace algo

} // end namespace maptk

#define INSTANTIATE_ALGORITHM_DEF(T) \
template class maptk::algo::algorithm_def<T>; \
namespace maptk \
{ \
template<> registrar<T>* registrar<T>::instance_ = 0; \
}

#endif // MAPTK_ALGO_ALGORITHM_TXX_
