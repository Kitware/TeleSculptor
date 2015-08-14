/*ckwg +29
 * Copyright 2014-2015 by Kitware, Inc.
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
 * \brief base algorithm function implementations
 */

#include "algorithm.h"

namespace maptk
{

namespace algo
{


/// Get this alg's \link maptk::config_block configuration block \endlink
config_block_sptr
algorithm
::get_configuration() const
{
  return config_block::empty_config(this->type_name());
}


algorithm_sptr
algorithm
::create(const std::string& type_name,
         const std::string& impl_name)
{
  std::string qualified_name = type_name + ":" + impl_name;
  algorithm_sptr inst = registrar::instance().find<algorithm>(qualified_name);
  if (!inst)
  {
    return inst;
  }
  return inst->base_clone();
}


std::vector<std::string>
algorithm
::registered_names(const std::string& type_name)
{
  if( type_name == "" )
  {
    return registrar::instance().registered_names<algorithm>();
  }
  std::vector<std::string> type_reg_names;
  const std::string prefix = type_name + ":";
  BOOST_FOREACH( std::string qual_name,
                 registrar::instance().registered_names<algorithm>() )
  {
    // if prefix is a prefix of qual_name, add it to the vector
    if (qual_name.length() >= prefix.length() &&
        std::equal(prefix.begin(), prefix.end(), qual_name.begin()))
    {
      type_reg_names.push_back(qual_name.substr(prefix.length()));
    }
  }
  return type_reg_names;
}


bool
algorithm
::has_type_name(const std::string& type_name)
{
  std::vector<std::string> valid_names = algorithm::registered_names(type_name);
  return !valid_names.empty();
}


bool
algorithm
::has_impl_name(const std::string& type_name,
                const std::string& impl_name)
{
  std::vector<std::string> valid_names = algorithm::registered_names(type_name);
  return std::find(valid_names.begin(), valid_names.end(), impl_name) != valid_names.end();
}

/// Helper function for properly getting a nested algorithm's configuration
void
algorithm
::get_nested_algo_configuration(std::string const& type_name,
                                std::string const& name,
                                config_block_sptr config,
                                algorithm_sptr nested_algo)
{
  config_block_description_t type_comment =
    "Algorithm to use for '" + name + "', which is of type '" + type_name + "'.\n"
    "Must be one of the following options:"
    ;
  std::string tmp_d;
  BOOST_FOREACH( std::string reg_name, algorithm::registered_names(type_name) )
  {
    type_comment += "\n\t- " + reg_name;
    std::string qualified_name = type_name + ":" + reg_name;
    tmp_d = registrar::instance().find<algorithm>(qualified_name)->description();
    if ( tmp_d != "" )
    {
      type_comment += " :: " + tmp_d;
    }
  }

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
void
algorithm
::set_nested_algo_configuration(std::string const& type_name,
                                std::string const& name,
                                config_block_sptr config,
                                algorithm_sptr &nested_algo)
{
  if(config->has_value(name + config_block::block_sep + "type"))
  {

    std::string iname = config->get_value<std::string>(name
                                                       + config_block::block_sep
                                                       + "type");
    if(algorithm::has_impl_name(type_name, iname))
    {
      nested_algo = algorithm::create(type_name, iname);
      nested_algo->set_configuration(
        config->subblock_view(name + config_block::block_sep + iname)
      );
    }
  }
}


/// Helper macro for checking that basic nested algorithm configuration is valid
bool
algorithm
::check_nested_algo_configuration(std::string const& type_name,
                                  std::string const& name,
                                  config_block_sptr config)
{
  const std::string type_key = name + config_block::block_sep + "type";
  if(!config->has_value(type_key))
  {
    std::cerr << "Configuration Failure: missing value "
              << type_key << std::endl;
    return false;
  }
  std::string iname = config->get_value<std::string>(type_key);
  if(!algorithm::has_impl_name(type_name, iname))
  {
    std::cerr << "Configuration Failure: invalid option\n"
              << "   " << type_key << " = "<< iname << "\n"
              << "   valid options are";
    BOOST_FOREACH( std::string reg_name, algorithm::registered_names(type_name) )
    {
      std::cerr << "\n      " << reg_name;
    }
    std::cerr << std::endl;
    return false;
  }
  // retursively check the configuration of the sub-algorithm
  std::string qualified_name = type_name + ":" + iname;
  if( !registrar::instance().find<algorithm>(qualified_name)->check_configuration(
          config->subblock_view(name + config_block::block_sep + iname)))
  {
    std::cerr << "Configuration Failure Backtrace: "
              << name + config_block::block_sep + iname << std::endl;
    return false;
  }
  return true;
}


} // end namespace algo

} // end namespace maptk
