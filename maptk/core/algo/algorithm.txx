/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_ALGO_ALGORITHM_TXX_
#define MAPTK_ALGO_ALGORITHM_TXX_

#include "algorithm.h"

#include <algorithm>
#include <iostream>
#include <sstream>
#include <typeinfo>

#include <maptk/core/exceptions/algorithm.h>
#include <maptk/core/registrar.h>

#include <boost/foreach.hpp>

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


/// Check the given name against registered implementation names
template <typename Self>
bool
algorithm_def<Self>
::check_impl_name(std::string const& impl_name)
{
  std::vector<std::string> valid_names = algorithm_def<Self>::registered_names();
  return std::find(valid_names.begin(), valid_names.end(), impl_name) != valid_names.end();
}


/// Return a \c config_block for all registered implementations
template <typename Self>
config_block_sptr
algorithm_def<Self>
::get_impl_configurations()
{
  config_block_sptr config = config_block::empty_config();
  BOOST_FOREACH( std::string impl_name, algorithm_def<Self>::registered_names() )
  {
    // create a clone of the impl in order to get access to its configuration,
    // merging it with the main config_block under a subblock that is the name
    // of the impl.
    config->subblock_view(impl_name)
          ->merge_config(algorithm_def<Self>::create(impl_name)->get_configuration());
  }
  return config;
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
