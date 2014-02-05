/**ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief Template implementations of OCV configuration helper methods
 */

#ifndef MAPTK_OCV_OCV_ALGO_TOOLS_TXX
#define MAPTK_OCV_OCV_ALGO_TOOLS_TXX

#include <iostream>
#include <sstream>
#include <vector>

#include <maptk/core/exceptions/config_block.h>

#include <boost/foreach.hpp>

namespace maptk
{

namespace ocv
{


namespace
{


/// Extract an OCV algorithm property and insert its value into a config_block
/**
 * \pre{type given to \c param_t must have a << stream operator defined
 *      for conversion to a string for storage in the \c config_block}
 *
 * \param algo_t  The algorithm type
 * \param param_t The type of the parameter being extracted and stored.
 * \param algo        The cv pointer to the algorithm object.
 * \param algo_name   The name given to the nested algorithm.
 * \param param_name  The name of the parameter to store
 * \param config      The \c config_block to store the parameter in.
 */
template <typename algo_t, typename param_t>
void ocv_algo_param_to_config(cv::Ptr<algo_t> algo,
                              std::string const& algo_name,
                              std::string const& param_name, 
                              config_block_sptr config)
{
  param_t param = algo->template get<param_t>(param_name);
  std::string param_descr = algo->paramHelp(param_name);

  std::ostringstream sstr;
  sstr << param;

  config->set_value(algo_name + config_block::block_sep + param_name,
                    sstr.str(),
                    param_descr);
}


/// Set a configuration parameter from a config_block to the nested algo
/**
 * \param algo_t  The algorithm type
 * \param param_t The type of the parameter being extracted and stored.
 * \param algo        The cv pointer to the algorithm object.
 * \param algo_name   The name given to the nested algorithm.
 * \param param_name  The name of the parameter to store
 * \param config      The \c config_block to store the parameter in.
 */
template <typename algo_t, typename param_t>
void config_to_ocv_algo_param(cv::Ptr<algo_t> algo,
                              std::string const& algo_name,
                              std::string const& param_name,
                              config_block_sptr config)
{
  param_t param = config->get_value<param_t>(
      algo_name + config_block::block_sep + param_name);
  algo->set(param_name, param);
}


/// Check that a parameter in a nested algo exists in the config_block
/**
 * Check that the \c config_block contains the proper variable as would have
 * been set by \c ocv_algo_param_to_config and that the extracted value can
 * be case to the algorithm parameter's expected type.
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
      return true;
    }
    catch (config_block_exception ex)
    { /* any exception is a check failure */ }
  }
  return false;
}


} // end anonymous namespace


/// Add nested OpenCV algorithm's configuration options to the given \c config
template <typename T>
void
get_nested_ocv_algo_configuration(std::string const& name,
                                  config_block_sptr config,
                                  cv::Ptr<T> algo)
{
  // How to use the get method on an algo:
  //    algo->template get<int>("some-name")
  //          !^^^^^^!
  //           needed
  using namespace std;

  vector<string> algo_params;
  algo->getParams(algo_params);

  cerr << "[get_nested_ocv_algo_configuration] Available params for "
       << algo->name() << ": "
       << endl;
  BOOST_FOREACH( string pname, algo_params)
  {
    cerr << "\t- '" << pname << "'" << endl;
    cerr << "\t\t- type id: " << algo->paramType(pname) << endl;

    int ptypeid = algo->paramType(pname);

    if (ptypeid == 0) // int
    {
      ocv_algo_param_to_config<T, int>(algo, name, pname, config);
    }
    else if (ptypeid == 1) // bool
    {
      ocv_algo_param_to_config<T, bool>(algo, name, pname, config);
    }
    else if (ptypeid == 2) // double
    {
      ocv_algo_param_to_config<T, double>(algo, name, pname, config);
    }
    else
    {
      cerr << "WARNING: [get_nested_ocv_algo_configuration] "
           << name << "(" << algo->info()->name() << ")->" << pname << " -- "
           << "Unexpected parameter type ID: " << ptypeid
           << endl;
    }
  }  
}


/// Set nested OpenCV algoruthm's parameters based on a given \c config
template <typename T>
void
set_nested_ocv_algo_configuration(std::string const& name,
                                  config_block_sptr config,
                                  cv::Ptr<T> algo)
{
  std::vector<std::string> algo_params;
  algo->getParams(algo_params);

  BOOST_FOREACH( std::string pname, algo_params )
  {
    int ptypeid = algo->paramType(pname);

    if (ptypeid == 0) // int
    {
      config_to_ocv_algo_param<T, int>(algo, name, pname, config);
    }
    else if (ptypeid == 1) // bool
    {
      config_to_ocv_algo_param<T, bool>(algo, name, pname, config);
    }
    else if (ptypeid == 2) // double
    {
      config_to_ocv_algo_param<T, double>(algo, name, pname, config);
    }
    else
    {
      std::cerr << "WARNING: [set_nested_ocv_algo_configuration] "
                << name << "(" << algo->info()->name() << ")->" << pname << " -- "
                << "Unexpected parameter type ID: " << ptypeid
                << std::endl;
    }
  }
}


/// Basic check of nested OpenCV algorithm configuration in the given \c config
template <typename T>
bool
check_nested_ocv_algo_configuration(std::string const& name,
                                    config_block_sptr config,
                                    cv::Ptr<T> algo)
{
  std::vector<std::string> algo_params;
  algo->getParams(algo_params);

  bool all_success = true;

  BOOST_FOREACH( std::string pname, algo_params )
  {
    int ptypeid = algo->paramType(pname);
    
    if (ptypeid == 0) // int
    {
      all_success &= check_ocv_algo_param_in_config<int>(name, pname, config);
    }
    else if (ptypeid == 1) // bool
    {
      all_success &= check_ocv_algo_param_in_config<bool>(name, pname, config);
    }
    else if (ptypeid == 2) // double
    {
      all_success &= check_ocv_algo_param_in_config<double>(name, pname, config);
    }
    else
    {
      std::cerr << "WARNING: [check_nested_ocv_algo_configuration] "
                << name << "(" << algo->info()->name() << ")->" << pname << " -- "
                << "Unexpected parameter type ID: " << ptypeid
                << std::endl;
    }
  }

  return all_success;
}


} // end namespace ocv

} // end namespace maptk

#endif // MAPTK_OCV_OCV_ALGO_TOOLS_TXX
