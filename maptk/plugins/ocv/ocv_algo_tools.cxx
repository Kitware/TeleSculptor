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
 * \brief Implementations of OCV configuration helper methods
 */

#include "ocv_algo_tools.h"

#include <iostream>
#include <vector>

#include <boost/algorithm/string/join.hpp>
#include <boost/foreach.hpp>


namespace kwiver {
namespace maptk {

namespace ocv
{


namespace
{


/// Extract an OCV algorithm property and insert its value into a vital::config_block
/**
 * \param param_t The type of the parameter being extracted and stored.
 * \param algo        The cv pointer to the algorithm object.
 * \param algo_name   The name given to the nested algorithm.
 * \param param_name  The name of the parameter to store
 * \param config      The \c vital::config_block to store the parameter in.
 */
template <typename param_t>
void ocv_algo_param_to_config(cv::Ptr<cv::Algorithm> algo,
                              std::string const& algo_name,
                              std::string const& param_name,
                              vital::config_block_sptr config)
{
  param_t param = algo->template get<param_t>(param_name);
  std::string param_descr = algo->paramHelp(param_name);
  //DEBUG
  //std::cerr << "\tGetting '" << algo_name << "' param '" << param_name << "': " << param << std::endl;
  config->set_value(algo_name + vital::config_block::block_sep + param_name,
                    param, param_descr);
}


/// Set a configuration parameter from a vital::config_block to the nested algo
/**
 * Checks if the ``algo_name:param_name`` key exists and if it can be extracted
 * as the templated type.
 *
 * If the given vital::config_block doesn't have the expected key, we will do nothing,
 * maintaining the default value in the algorithm.
 *
 * \param param_t The type of the parameter being extracted and stored.
 * \param algo        The cv pointer to the algorithm object.
 * \param algo_name   The name given to the nested algorithm.
 * \param param_name  The name of the parameter to store
 * \param config      The \c vital::config_block to store the parameter in.
 */
template <typename param_t>
void config_to_ocv_algo_param(cv::Ptr<cv::Algorithm> algo,
                              std::string const& algo_name,
                              std::string const& param_name,
                              vital::config_block_sptr config)
{
  vital::config_block_key_t param_key = algo_name + vital::config_block::block_sep + param_name;
  if (config->has_value(param_key))
  {
    param_t param = config->get_value<param_t>(param_key);
    //DEBUG
    //std::cerr << "\tSetting '" << algo_name << "' param '" << param_name << "': " << param << std::endl;
    algo->set(param_name, param);
  }
  // else leaving algo default alone
}


/// Check that a parameter in a nested algo exists in the vital::config_block
/**
 * All open CV algorithm parameters are optional (they all have default
 * values). Because of this, we don't fail if the paramter is not in the
 * given \c vital::config_block. We only fail if the parameter extraction from the
 * \c vital::config_block fails.
 *
 * \param param_t The type of the parameter being extracted and stored.
 * \param algo_name   The name given to the nested algorithm.
 * \param param_name  The name of the parameter to store
 * \param config      The \c vital::config_block to store the parameter in.
 */
template <typename param_t>
bool check_ocv_algo_param_in_config(std::string const& algo_name,
                                    std::string const& param_name,
                                    vital::config_block_sptr config)
{
  vital::config_block_key_t key = algo_name + vital::config_block::block_sep + param_name;
  if (config->has_value(key))
  {
    try
    {
      param_t test_val = config->get_value<param_t>(key);
      (void) test_val;
      //DEBUG
      //std::cerr << "\tChecking '" << algo_name << "' param '" << param_name << "': " << test_val << std::endl;
    }
    catch (vital::config_block_exception ex)
    {
      return false;
    }
  }
  return true;
}


} // end anonymous namespace


/// Add nested OpenCV algorithm's configuration options to the given \c config
void
get_nested_ocv_algo_configuration(std::string const& name,
                                  vital::config_block_sptr config,
                                  cv::Ptr<cv::Algorithm> algo)
{
  using namespace std;
  vital::config_block_description_t type_descr =
    "The OpenCV cv::Algorithm type to use for '" + name + "'.";

  // we were given a pointer to an instantiated algorithm
  if (! algo.empty())
  {
    std::string impl_name = algo->info()->name();
    config->set_value(name + vital::config_block::block_sep + type_token,
                      impl_name, type_descr);

    vector<string> algo_params;
    algo->getParams(algo_params);

    BOOST_FOREACH( string pname, algo_params)
    {
      int ptypeid = algo->paramType(pname);
      switch(ptypeid)
      {
        case 1: // bool
          ocv_algo_param_to_config<bool>(algo, impl_name, pname, config->subblock_view(name));
          break;
        case 0: // int
        //case 8: // unsigned int  WARNING: These are unsafe type conversion
        //case 9: // uint64        given the current method of setting
        //                         parameters. Insufficient pass-throughs in
        //                         cv::Algorithm class (will have to access
        //                         underlying cv::AlgorithmInfo instance
        //                         directly).
        case 10:// short
        case 11:// uchar
          ocv_algo_param_to_config<int>(algo, impl_name, pname, config->subblock_view(name));
          break;
        case 2: // double
        case 7: // float
          ocv_algo_param_to_config<double>(algo, impl_name, pname, config->subblock_view(name));
          break;
        case 3: // std::string
          ocv_algo_param_to_config<std::string>(algo, impl_name, pname, config->subblock_view(name));
          break;
        case 6: // cv::Algorithm
          {
            // Recursively call this method for an OpenCV algorithm's nested
            // algorithm. Options are laid out just like a nested OCV
            // algorithm of MAPTK algorithm.
            cv::Ptr<cv::Algorithm> nested_algo = algo->get<cv::Algorithm>(pname);
            get_nested_ocv_algo_configuration(
                pname,
                config->subblock_view(name + vital::config_block::block_sep + impl_name),
                nested_algo
                );
            break;
          }
        default:
          cerr << "WARNING: [get_nested_ocv_algo_configuration] "
             << name << "(" << algo->info()->name() << ")->" << pname << " -- "
             << "Unexpected parameter type ID: " << ptypeid
             << endl;
          break;
      }
    }
  }
  else
  {
    config->set_value(name + vital::config_block::block_sep + type_token, "", type_descr);
  }
}


namespace helper_
{


// (Helper) Set nested OpenCV algorithm's parameters based on a given \c config
void
set_nested_ocv_algo_configuration_helper(std::string const& name,
                                         vital::config_block_sptr config,
                                         cv::Ptr<cv::Algorithm> &algo)
{
  // Only proceed if a valid algorithm was created
  if (!algo.empty())
  {
    std::string impl_name = algo->info()->name();

    // scan through algo parameters, settings ones that we can encode in the vital::config_block
    std::vector<std::string> algo_params;
    algo->getParams(algo_params);
    BOOST_FOREACH( std::string const pname, algo_params )
    {
      int ptypeid = algo->paramType(pname);
      switch(ptypeid)
      {
        case 1: // bool
          config_to_ocv_algo_param<bool>(algo, impl_name, pname, config->subblock_view(name));
          break;
        case 0: // int
        //case 8: // unsigned int  WARNING: These are unsafe type conversion
        //case 9: // uint64        given the current method of setting
        //                         parameters. Insufficient pass-throughs in
        //                         cv::Algorithm class (will have to access
        //                         underlying cv::AlgorithmInfo instance
        //                         directly).
        case 10:// short
        case 11:// uchar
          config_to_ocv_algo_param<int>(algo, impl_name, pname, config->subblock_view(name));
          break;
        case 2: // double
        case 7: // float
          config_to_ocv_algo_param<double>(algo, impl_name, pname, config->subblock_view(name));
          break;
        case 3: // std::string
          config_to_ocv_algo_param<std::string>(algo, impl_name, pname, config->subblock_view(name));
          break;
        case 6: // cv::Algorithm
          {
            // recursively set nested args on nested algo
            cv::Ptr<cv::Algorithm> nested_algo = algo->get<cv::Algorithm>(pname);
            set_nested_ocv_algo_configuration(
                pname,
                config->subblock_view(name + vital::config_block::block_sep + impl_name),
                nested_algo
                );
            algo->set(pname, nested_algo);
            break;
          }
        default:
          std::cerr << "WARNING: [set_nested_ocv_algo_configuration_helper] "
                << name << "(" << algo->info()->name() << ")->" << pname << " -- "
                << "Unexpected parameter type ID: " << ptypeid
                << std::endl;
      }
    }
  }
}


/// Basic check of nested OpenCV algorithm configuration in the given \c config
bool
check_nested_ocv_algo_configuration_helper(std::string const& name,
                                           vital::config_block_sptr config,
                                           cv::Ptr<cv::Algorithm> algo)
{
  std::string impl_name = algo->info()->name();

  std::vector<std::string> algo_params;
  algo->getParams(algo_params);

  bool all_success = true;
  int ptypeid;
  BOOST_FOREACH( std::string pname, algo_params )
  {
    ptypeid = algo->paramType(pname);
    switch(ptypeid)
    {
      case 1: // bool
        all_success =
          all_success && check_ocv_algo_param_in_config<bool>(impl_name, pname, config->subblock_view(name));
        break;
      case 0: // int
      //case 8: // unsigned int  WARNING: These are unsafe type conversion
      //case 9: // uint64        given the current method of setting
      //                         parameters. Insufficient pass-throughs in
      //                         cv::Algorithm class (will have to access
      //                         underlying cv::AlgorithmInfo instance
      //                         directly).
      case 10:// short
      case 11:// uchar
        all_success =
          all_success && check_ocv_algo_param_in_config<int>(impl_name, pname, config->subblock_view(name));
        break;
      case 2: // double
        case 7: // float
        all_success =
          all_success && check_ocv_algo_param_in_config<double>(impl_name, pname, config->subblock_view(name));
        break;
      case 3: // std::string
        all_success =
          all_success && check_ocv_algo_param_in_config<std::string>(impl_name, pname, config->subblock_view(name));
        break;
      case 6: // cv::Algorithm
        {
          all_success = all_success && check_nested_ocv_algo_configuration
            <cv::Algorithm>(
              pname,
              config->subblock_view(name + vital::config_block::block_sep + impl_name)
              );
          break;
        }
      default:
        std::cerr << "WARNING: [check_nested_ocv_algo_configuration] "
                  << name << "(" << algo->info()->name() << ")->" << pname << " -- "
                  << "Unexpected parameter type ID: " << ptypeid
                  << std::endl;
    }
  }
  return all_success;
}


template<>
cv::Ptr<cv::Algorithm> create_ocv_algo<cv::Algorithm>(std::string const& impl_name)
{
  return cv::Algorithm::create<cv::Algorithm>(impl_name);
}


} // end namespace helper_

} // end namespace ocv

} // end namespace maptk
} // end namespace kwiver
