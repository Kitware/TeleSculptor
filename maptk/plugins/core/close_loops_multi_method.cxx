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
 * \brief Implementation of \link maptk::algo::close_loops_multi_method
 *        close_loops_multi_method \endlink
 */

#include "close_loops_multi_method.h"

#include <algorithm>
#include <iostream>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>

#include <vital/algorithm.h>
#include <vital/exceptions/algorithm.h>


namespace maptk
{

namespace core
{

using namespace kwiver::vital;

// Return IDs of all methods labels.
std::vector< std::string >
method_names( unsigned count )
{
  std::vector< std::string > output;

  for( unsigned i = 0; i < count; i++ )
  {
    output.push_back( "method" + boost::lexical_cast<std::string>( i+1 ) );
  }

  return output;
}


close_loops_multi_method
::close_loops_multi_method()
: count_( 1 ),
  methods_( 1 )
{
}


close_loops_multi_method
::close_loops_multi_method(const close_loops_multi_method& other)
: count_( other.count_ ),
  methods_( other.methods_.size() )
{
}


/// Returns implementation description string
std::string
close_loops_multi_method
::description() const
{
  return "Iteratively run multiple loop closure algorithms";
}


  kwiver::config_block_sptr
close_loops_multi_method
::get_configuration() const
{
  // Get base config from base class
  kwiver::config_block_sptr config = algorithm::get_configuration();

  // Internal parameters
  config->set_value( "count", count_, "Number of close loops methods we want to use." );

  // Sub-algorithm implementation name + sub_config block
  std::vector< std::string > method_ids = method_names( count_ );

  for( unsigned i = 0; i < method_ids.size(); i++ )
  {
    close_loops::get_nested_algo_configuration( method_ids[i], config, methods_[i] );
  }

  return config;
}


void
close_loops_multi_method
::set_configuration( kwiver::config_block_sptr in_config )
{
  // Starting with our generated config_block to ensure that assumed values are present
  // An alternative is to check for key presence before performing a get_value() call.
  kwiver::config_block_sptr config = this->get_configuration();
  config->merge_config( in_config );

  // Parse count parameter
  count_ = config->get_value<unsigned>( "count" );
  methods_.resize( count_ );

  // Parse methods
  std::vector<std::string> method_ids = method_names( count_ );

  for( unsigned i = 0; i < method_ids.size(); i++ )
  {
    close_loops::set_nested_algo_configuration( method_ids[i], config, methods_[i] );
  }
}


bool
close_loops_multi_method
::check_configuration( kwiver::config_block_sptr config ) const
{
  std::vector<std::string> method_ids = method_names( config->get_value<unsigned>( "count" ) );

  for( unsigned i = 0; i < method_ids.size(); i++ )
  {
    if( !close_loops::check_nested_algo_configuration( method_ids[i], config ) )
    {
      return false;
    }
  }

  return true;
}


track_set_sptr
close_loops_multi_method
::stitch( frame_id_t frame_number, track_set_sptr input,
          image_container_sptr image, image_container_sptr mask ) const
{
  track_set_sptr updated_set = input;

  for( unsigned i = 0; i < methods_.size(); i++ )
  {
    updated_set = methods_[i]->stitch( frame_number, updated_set, image, mask );
  }

  return updated_set;
}


} // end namespace core

} // end namespace maptk
