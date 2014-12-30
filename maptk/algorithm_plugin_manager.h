/*ckwg +29
 * Copyright 2014 by Kitware, Inc.
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
 * \brief Singleton manager of plug-in stuff
 */

#ifndef MAPTK_ALGORITHM_PLUGIN_MANAGER_H_
#define MAPTK_ALGORITHM_PLUGIN_MANAGER_H_

#include <string>
#include <vector>

#include <boost/shared_ptr.hpp>

#include <maptk/config.h>
#include <maptk/types.h>


namespace maptk
{


/**
 * Plugin Manager for algorithm implementation extensions
 */
class MAPTK_LIB_EXPORT algorithm_plugin_manager
{
public:
  /// Get the reference to the singleton instance of this class
  static algorithm_plugin_manager& instance();

  /// (Re)Load plugin libraries found along current search paths
  /**
   * \param name If a name is provided, we will only load plugins whose name
   *             corresponds to the name given. If no plugins with the given
   *             name are found, nothing is loaded.
   */
  void register_plugins( std::string name = std::string() );

  /// Add an additional directory to search for plugins in.
  /**
   * Directory paths that don't exist will simply be ignored.
   *
   * \param dirpath Path to the directory to add to the plugin search path
   */
  void add_search_path(path_t dirpath);

private:
  class impl;
  impl *impl_;

  /// Private constructor
  /**
   * The singleton instance of this class should only be accessed via the
   * ``instance()`` static method.
   */
  algorithm_plugin_manager();
  /// private deconstructor (singleton)
  virtual ~algorithm_plugin_manager();
  /// private copy constructor (singleton)
  algorithm_plugin_manager(algorithm_plugin_manager const&);
  /// private assignment operator (singleton)
  algorithm_plugin_manager& operator=(algorithm_plugin_manager const&);
};


} // end maptk namespace

#endif // MAPTK_ALGORITHM_PLUGIN_MANAGER_H_
