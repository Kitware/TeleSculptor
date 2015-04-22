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
 * \brief C interface to maptk::algorithm_plugin_manager
 */

#ifndef MAPTK_C_ALGORITHM_PLUGIN_MANAGER_H_
#define MAPTK_C_ALGORITHM_PLUGIN_MANAGER_H_


#ifdef __cplusplus
extern "C"
{
#endif

#include <maptk/c/config.h>


/// (Re)Load plugin modules found along current search paths
MAPTK_C_EXPORT
void maptk_apm_register_plugins();


/// (Re)Load specific plugin module
/**
 * \param name Find and load the plugins by the given name. If no plugins with
 *             the given name are found, nothing is loaded.
 */
MAPTK_C_EXPORT
void maptk_apm_register_single_plugin( char const *name );


/// Add an additional directory to search for plugins in
/**
 * Directory paths that don't exist will simply be ignored.
 *
 * \param dirpath Path to a directory to add to the plugin search path.
 */
MAPTK_C_EXPORT
void maptk_apm_add_search_path( char const *dirpath );


/// Get a list of registered module name strings
/**
 * A module's name is defined as the filename minus the standard platform
 * module library suffix. For example, on Windows, if a module library was
 * named ``maptk_foo.dll``, the module's name would be "maptk_foo". Similarly
 * on a unix system, ``maptk_bar.so`` would have the name "maptk_bar".
 *
 * We are expecting that the \p length and \p keys parameters will be passed
 * by reference by the user as they are dereferenced within the function for
 * value assignment.
 *
 * \param[out] length Pointer to an unsigned int into which we return the
 *                    number of registered module names.
 * \param[out] names Pointer to a string list into which we allocate and assign
 *                   registered module names.
 */
MAPTK_C_EXPORT
void maptk_apm_registered_module_names( unsigned int *length,
                                        char ***names );


#ifdef __cplusplus
}
#endif


#endif // MAPTK_C_ALGORITHM_PLUGIN_MANAGER_H_
