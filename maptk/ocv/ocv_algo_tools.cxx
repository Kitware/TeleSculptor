/**ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief Template implementations of OCV configuration helper methods
 */

#include "ocv_algo_tools.h"

#include <iostream>
#include <vector>

#include <boost/algorithm/string/join.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>

namespace maptk
{

namespace ocv
{


namespace
{


/// Extract an OCV algorithm property and insert its value into a config_block
/**
 * \param param_t The type of the parameter being extracted and stored.
 * \param algo        The cv pointer to the algorithm object.
 * \param algo_name   The name given to the nested algorithm.
 * \param param_name  The name of the parameter to store
 * \param config      The \c config_block to store the parameter in.
 */
template <typename param_t>
void ocv_algo_param_to_config(cv::Ptr<cv::Algorithm> algo,
                              std::string const& algo_name,
                              std::string const& param_name,
                              config_block_sptr config)
{
  param_t param = algo->template get<param_t>(param_name);
  std::string param_descr = algo->paramHelp(param_name);
  //DEBUG
  std::cerr << "\tGetting '" << algo_name << "' param '" << param_name << "': " << param << std::endl;
  config->set_value(algo_name + config_block::block_sep + param_name,
                    boost::lexical_cast<std::string>(param),
                    param_descr);
}


/// Set a configuration parameter from a config_block to the nested algo
/**
 * Checks if the ``algo_name:param_name`` key exists and if it can be extracted
 * as the templated type.
 *
 * If the given config_block doesn't have the expected key, we will do nothing,
 * maintaining the default value in the algorithm.
 *
 * \param param_t The type of the parameter being extracted and stored.
 * \param algo        The cv pointer to the algorithm object.
 * \param algo_name   The name given to the nested algorithm.
 * \param param_name  The name of the parameter to store
 * \param config      The \c config_block to store the parameter in.
 */
template <typename param_t>
void config_to_ocv_algo_param(cv::Ptr<cv::Algorithm> algo,
                              std::string const& algo_name,
                              std::string const& param_name,
                              config_block_sptr config)
{
  config_block_key_t param_key = algo_name + config_block::block_sep + param_name;
  if (config->has_value(param_key))
  {
    param_t param = config->get_value<param_t>(param_key);
    //DEBUG
    std::cerr << "\tSetting '" << algo_name << "' param '" << param_name << "': " << param << std::endl;
    algo->set(param_name, param);
  }
  // else leaving algo default alone
}


/// Check that a parameter in a nested algo exists in the config_block
/**
 * All open CV algorithm parameters are optional (they all have default
 * values). Because of this, we don't fail if the paramter is not in the
 * given \c config_block. We only fail if the parameter extraction from the
 * \c config_block fails.
 *
 * \param param_t The type of the parameter being extracted and stored.
 * \param algo_name   The name given to the nested algorithm.
 * \param param_name  The name of the parameter to store
 * \param config      The \c config_block to store the parameter in.
 */
template <typename param_t>
bool check_ocv_algo_param_in_config(std::string const& algo_name,
                                    std::string const& param_name,
                                    config_block_sptr config)
{
  config_block_key_t key = algo_name + config_block::block_sep + param_name;
  if (config->has_value(key))
  {
    try
    {
      param_t test_val = config->get_value<param_t>(key);
      (void) test_val;
      //DEBUG
      std::cerr << "\tChecking '" << algo_name << "' param '" << param_name << "': " << test_val << std::endl;
    }
    catch (config_block_exception ex)
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
                                  config_block_sptr config,
                                  cv::Ptr<cv::Algorithm> algo)
{
  using namespace std;

  // we were given a pointer to an instantiated algorithm
  if (! algo.empty())
  {
    std::string impl_name = algo->info()->name();
    config->set_value(name + config_block::block_sep + type_token,
                      impl_name);

    vector<string> algo_params;
    algo->getParams(algo_params);

    BOOST_FOREACH( string pname, algo_params)
    {
      int ptypeid = algo->paramType(pname);
      switch(ptypeid)
      {
        case 0: // int
          ocv_algo_param_to_config<int>(algo, impl_name, pname, config->subblock_view(name));
          break;
        case 1: // bool
          ocv_algo_param_to_config<bool>(algo, impl_name, pname, config->subblock_view(name));
          break;
        case 2: // double
          ocv_algo_param_to_config<double>(algo, impl_name, pname, config->subblock_view(name));
          break;
        case 6: // cv::Algorithm
          {
            // Recursively call this method for an OpenCV algorithm's nested
            // algorithm. Options are laid out just like a nested OCV
            // algorithm of MAPTK algorithm.
            cv::Ptr<cv::Algorithm> nested_algo = algo->get<cv::Algorithm>(pname);
            get_nested_ocv_algo_configuration(
                pname,
                config->subblock_view(name + config_block::block_sep + impl_name),
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
    config->set_value(name + config_block::block_sep + type_token, "");
  }
}


namespace helper_
{


// (Helper) Set nested OpenCV algorithm's parameters based on a given \c config
void
set_nested_ocv_algo_configuration_helper(std::string const& name,
                                         config_block_sptr config,
                                         cv::Ptr<cv::Algorithm> &algo)
{
  // Only proceed if a valid algorithm was created
  if (!algo.empty())
  {
    std::string impl_name = algo->info()->name();

    // scan through algo parameters, settings ones that we can encode in the config_block
    std::vector<std::string> algo_params;
    algo->getParams(algo_params);
    BOOST_FOREACH( std::string const pname, algo_params )
    {
      int ptypeid = algo->paramType(pname);
      switch(ptypeid)
      {
        case 0: // int
          config_to_ocv_algo_param<int>(algo, impl_name, pname, config->subblock_view(name));
          break;
        case 1: // bool
          config_to_ocv_algo_param<bool>(algo, impl_name, pname, config->subblock_view(name));
          break;
        case 2: // double
          config_to_ocv_algo_param<double>(algo, impl_name, pname, config->subblock_view(name));
          break;
        case 6: // cv::Algorithm
          {
            // recursively set nested args on nested algo
            cv::Ptr<cv::Algorithm> nested_algo = algo->get<cv::Algorithm>(pname);
            set_nested_ocv_algo_configuration(
                pname,
                config->subblock_view(name + config_block::block_sep + impl_name),
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
                                           config_block_sptr config,
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
      case 0: // int
        all_success =
          all_success && check_ocv_algo_param_in_config<int>(impl_name, pname, config->subblock_view(name));
        break;
      case 1: // bool
        all_success =
          all_success && check_ocv_algo_param_in_config<bool>(impl_name, pname, config->subblock_view(name));
        break;
      case 2: // double
        all_success =
          all_success && check_ocv_algo_param_in_config<double>(impl_name, pname, config->subblock_view(name));
        break;
      case 6: // cv::Algorithm
        {
          all_success = all_success && check_nested_ocv_algo_configuration
            <cv::Algorithm>(
              pname,
              config->subblock_view(name + config_block::block_sep + impl_name)
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
