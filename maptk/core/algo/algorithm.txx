/*ckwg +29
 * Copyright 2013-2014 by Kitware, Inc.
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
  std::string const key( name + config_block::block_sep + "type" );
  std::string iname;

  if(config->has_value( key ) )
  {
    iname = config->get_value<std::string>( key );
    if(algorithm_def<Self>::has_impl_name(iname))
    {
      // \todo add log message
      std::cerr << "DEBUG - configuring \"" << name << "\" with algorithm type \""
                << iname << "\"\n";

      nested_algo = algorithm_def<Self>::create(iname);
      nested_algo->set_configuration(
        config->subblock_view(name + config_block::block_sep + iname)
      );
    }
    else
    {
      // \todo add log message, is this an error?
      std::cerr << "WARNING - impl name:\"" << iname << "\" not supported, from key \""
                << key << "\"\n";
    }
  }
  else
  {
    /// \todo add log message
    std::cerr << "WARNING - config key: \"" << key <<  "\" not found\n";
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
    // \todo add log message DEBUG
    std::cerr << "DEBUG - config block does not contain \""
              << name << config_block::block_sep << "type\"\n";
    return false;
  }
  std::string iname = config->get_value<std::string>(name + config_block::block_sep + "type");
  if(!algorithm_def<Self>::has_impl_name(iname))
  {
    // \todo add log message DEBUG
    std::cerr << "DEBUG - algorithm does not have implementation name \"" << iname
          << "\"\n";
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
