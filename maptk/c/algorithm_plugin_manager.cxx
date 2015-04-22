/*ckwg +29
 * Copyright 2015 by Kitware, Inc.
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
 * \brief C interface to maptk::algorithm_plugin_manager implementation
 */

#include "algorithm_plugin_manager.h"

#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include <maptk/algorithm_plugin_manager.h>

#include <maptk/c/helpers/c_utils.h>


/// (Re)Load plugin modules found along current search paths
void maptk_apm_register_plugins()
{
  STANDARD_CATCH(
    "C::apm::register_plugins", 0,
    maptk::algorithm_plugin_manager::instance().register_plugins();
  );
}


/// (Re)Load specific plugin module
void maptk_apm_register_single_plugin( char const *name )
{
  STANDARD_CATCH(
    "C::apm::register_single_plugin", 0,
    maptk::algorithm_plugin_manager::instance().register_plugins( name );
  );
}


/// Add an additional directory to search for plugins in
void maptk_apm_add_search_path( char const *dirpath )
{
  STANDARD_CATCH(
    "C::apm::add_search_path", 0,
    maptk::algorithm_plugin_manager::instance().add_search_path( dirpath );
  );
}


/// Get a list of registered module name strings
void maptk_apm_registered_module_names( unsigned int *length,
                                        char ***names )
{
  STANDARD_CATCH(
    "C::apm::registered_module_names", 0,

    if ( length == 0 || names == 0 )
    {
      throw maptk::invalid_value("One or both provided output parameters "
                                 "were a NULL pointer.");
    }

    std::vector<std::string> module_names =
        maptk::algorithm_plugin_manager::instance().registered_module_names();
    maptk_c::make_string_list( module_names, *length, *names );
  );
}
