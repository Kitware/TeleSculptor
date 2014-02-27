/**ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief implementation of algorithm/_def/_impl templated methods
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

#include <boost/algorithm/string/join.hpp>
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
::has_impl_name(std::string const& impl_name)
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
          ->merge_config(registrar<Self>::find(impl_name)->get_configuration());
  }
  return config;
}


/// Helper function for properly getting a nested algorithm's configuration
template <typename Self>
void
algorithm_def<Self>
::get_nested_algo_configuration(std::string const& name,
                                config_block_sptr config,
                                base_sptr nested_algo)
{
  config_block_description_t type_comment =
    "Algorithm to use for '" + name + "'.\n"
    "Must be one of the following options:\n\t- "
    + boost::algorithm::join(algorithm_def<Self>::registered_names(), "\n\t- ")
    ;

  if(nested_algo)
  {
    config->set_value(name + config_block::block_sep + "type",
                      nested_algo->impl_name(),
                      type_comment);
    config->subblock_view(name + config_block::block_sep + nested_algo->impl_name())
          ->merge_config(nested_algo->get_configuration());
  }
  else if (!config->has_value(name + config_block::block_sep + "type"))
  {
    config->set_value(name + config_block::block_sep + "type",
                      "",
                      type_comment);
  }
}


/// Helper macro for properly setting a nested algorithm's configuration
template <typename Self>
void
algorithm_def<Self>
::set_nested_algo_configuration(std::string const& name,
                                config_block_sptr config,
                                base_sptr &nested_algo)
{
  if(config->has_value(name + config_block::block_sep + "type"))
  {

    std::string iname = config->get_value<std::string>(name
                                                       + config_block::block_sep
                                                       + "type");
    if(algorithm_def<Self>::has_impl_name(iname))
    {
      nested_algo = algorithm_def<Self>::create(iname);
      nested_algo->set_configuration(
        config->subblock_view(name + config_block::block_sep + iname)
      );
    }
  }
}


/// Helper macro for checking that basic nested algorithm configuration is valid
template <typename Self>
bool
algorithm_def<Self>
::check_nested_algo_configuration(std::string const& name,
                                  config_block_sptr config)
{
  if(!config->has_value(name + config_block::block_sep + "type"))
  {
    return false;
  }
  std::string iname = config->get_value<std::string>(name + config_block::block_sep + "type");
  if(!algorithm_def<Self>::has_impl_name(iname))
  {
    return false;
  }
  // retursively check the configuration of the sub-algorithm
  return registrar<Self>::find(iname)->check_configuration(
    config->subblock_view(name + config_block::block_sep + iname)
  );
}

} // end namespace algo

} // end namespace maptk


/// \cond DoxygenSuppress
#define INSTANTIATE_ALGORITHM_DEF(T) \
template class maptk::algo::algorithm_def<T>; \
namespace maptk \
{ \
template<> registrar<T>* registrar<T>::instance_ = 0; \
}
/// \endcond

#endif // MAPTK_ALGO_ALGORITHM_TXX_
