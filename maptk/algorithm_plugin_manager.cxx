/*ckwg +29
 * Copyright 2014-2016 by Kitware, Inc.
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
 * \brief Algorithm plugin manager implementation
 */

#include "algorithm_plugin_manager.h"

#include <map>
#include <string>
#include <vector>

#if defined(_WIN32) || defined(_WIN64)
# include <windows.h>
#else
# include <dlfcn.h>
#endif

#include <boost/algorithm/string/predicate.hpp>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>

#include <maptk/logging_macros.h>
#include <maptk/registrar.h>

#ifndef BUILD_SHARED_LIBS
# include <maptk/algorithm_plugin_manager_static.h>
#endif


namespace bfs = boost::filesystem;


namespace maptk
{


namespace // anonymous
{


/// Execute specific code provided based on current platform
#if defined(_WIN32) || defined(_WIN64)
# define HANDLE_PLATFORM(windows_code, unix_code) windows_code
#else
# define HANDLE_PLATFORM(windows_code, unix_code) unix_code
#endif

/// Use __extension__ modifier if using GNUC.
/// See: https://trac.osgeo.org/qgis/ticket/234#comment:17
#ifdef __GNUC__
# define GNUC_EXTENSION __extension__
#else
# define GNUC_EXTENSION
#endif


HANDLE_PLATFORM(
  /* windows */
  typedef HMODULE library_t;
  typedef FARPROC function_t;
,
  /* unix */
  typedef void* library_t;
  typedef void* function_t;
);
typedef int (*register_impls_func_t)(registrar&);


// String name of the private interface function.
// See source file located @ CMake/templates/cxx/plugin_shell.cxx
static std::string const register_function_name = std::string("private_register_algo_impls");
// Platform specific plugin library file (set as compile definition in CMake)
static std::string const shared_library_suffix = std::string(SHARED_LIB_SUFFIX);

// Default module directory locations. Values defined in CMake configuration.
static maptk::path_t const default_plugin_dir_build = maptk::path_t(DEFAULT_PLUGIN_DIR_BUILD),
                           default_plugin_dir_install = maptk::path_t(DEFAULT_PLUGIN_DIR_INSTALL),
                           extra_path = maptk::path_t(EXTRA_MODULE_PATH);
bool const use_build_plugin_dir = USE_BUILD_PLUGIN_DIR;

} // end anonymous namespace


// ===========================================================================
// PluginManager Private Implementation
// ---------------------------------------------------------------------------

class algorithm_plugin_manager::impl
{
// Memeber Variables ---------------------------------------------------------
public:
  /// Paths in which to search for module libraries
  typedef std::vector<path_t> search_paths_t;
  search_paths_t search_paths_;
  /// module libraries already loaded, keyed on filename
  typedef std::map< std::string, path_t > registered_modules_t;
  registered_modules_t registered_modules_;

// Member functions ----------------------------------------------------------
public:
  impl()
    : search_paths_()
  {}


  /// Attempt loading algorithm implementations from all known search paths
  void load_from_search_paths( std::string name = std::string() )
  {
    LOG_DEBUG("algorithm_plugin_manager::impl::load_from_search_paths",
              "Loading plugins in search paths");
    // TODO: Want a way to hook into an environment variable / config file here
    //       for additional search path extension
    //       - Search order: setInCode -> EnvVar -> configFile -> defaults
    //       - create separate default_search_paths_ member var for separate storage
    BOOST_FOREACH( path_t module_dir, this->search_paths_ )
    {
      load_modules_in_directory(module_dir, name);
    }
  }


  /// Attempt loading algorithm implementations from all plugin modules in dir
  /**
   * If the given path is not a valid directory, we emit a warning message
   * and return without doing anything else.
   */
  void load_modules_in_directory(path_t dir_path, std::string name = std::string() )
  {
    // Check given path for validity
    // Preventing load from current directory via empty string (security)
    if (dir_path.empty())
    {
      LOG_DEBUG( "algorithm_plugin_manager::impl::load_modules_in_directory",
                 "Empty directory in the search path. Ignoring." );
      return;
    }
    if (!bfs::exists(dir_path))
    {
      LOG_DEBUG( "algorithm_plugin_manager::impl::load_modules_in_directory",
                 "Path " << dir_path << " doesn't exist. Ignoring." );
      return;
    }
    if (!bfs::is_directory(dir_path))
    {
      LOG_DEBUG( "algorithm_plugin_manager::impl::load_modules_in_directory",
                 "Path " << dir_path << " is not a directory. Ignoring." );
      return;
    }

    // Iterate over search-path directories, attempting module load on elements
    // that end in the configured library suffix.
    LOG_DEBUG("algorithm_plugin_manager::impl::load_modules_in_directory",
              "Loading modules in directory: " << dir_path);
    bfs::directory_iterator dir_it(dir_path);
    while (dir_it != bfs::directory_iterator())
    {
      bfs::directory_entry const e = *dir_it;

      // Accept this file as a module to check if it has the correct library
      // suffix and matches a provided module name if one was provided.
      if ( boost::ends_with(e.path().string(), shared_library_suffix)
           && ( name.size() == 0 || e.path().stem().string() == name ) )
      {
        // Check that we're looking a file
        if (e.status().type() == bfs::regular_file)
        {
          register_from_module(e.path());
        }
        else
        {
          LOG_WARN("algorithm_plugin_manager::impl::load_modules_in_directory",
                   "Encountered a directory entry " << e.path() <<
                   " which ends with the expected suffix, but is not" <<
                   " a file");
        }
      }

      ++dir_it;
    }
  }


  /// Find and execute algorithm impl registration call-back in the module
  /**
   * If the expected registration function is found, it is executed. If not,
   * the library is closed and nothing further is performed as we assume this
   * plugis just didn't provide any algorithm implementation extensions.
   *
   * \param module_path Filesystem path to the module library file to load.
   * \returns True of the module was loaded and used in some manner (i.e. still
   *          loaded). False if the module could not be loaded there was no
   *          successful interfacing.
   *
   * TODO: Just use exception handing instead of return codes and booleans.
   */
  bool register_from_module(path_t module_path)
  {
    LOG_DEBUG("algorithm_plugin_manager::impl::register_from_module",
              "Starting plug-in interfacing for module file: " << module_path);

    //
    // Attempting module load
    //
    library_t library = NULL;
    std::string err_str;
    HANDLE_PLATFORM(
      /* windows */
      library = LoadLibrary( module_path.string().c_str() );
      // TODO: Catch platform specific error string on failure
      ,
      /* unix */
      dlerror();
      library = dlopen( module_path.string().c_str(), RTLD_LAZY | RTLD_GLOBAL );
      if (!library)
      {
        err_str = std::string(dlerror());
      }
    );

    if (!library)
    {
      LOG_ERROR("algorithm_plugin_manager::impl::register_from_module",
                "Failed to open module library " << module_path <<
                " (error: " << err_str << ")");
      return false; // TODO: Throw exception here?
    }
    LOG_DEBUG("algorithm_plugin_manager::impl::register_from_module",
              "Loaded module: " << library);

    //
    // Attempt to load each available interface here
    //
    bool module_used = false;

    { // Algorithm Implementation interface

      // If interface function not found, we assume this plugin doesn't provide
      // any algorithm implementations and close the library. We otherwise keep
      // it open if we are going to use things from it.
      LOG_DEBUG("algorithm_plugin_manager::impl::register_from_module",
                "Looking for algorithm impl registration function: "
                << register_function_name.c_str());
      function_t register_func = NULL;
      HANDLE_PLATFORM(
        /* Windows */
        register_func = GetProcAddress( library, register_function_name.c_str() );
        ,
        /* Unix */
        register_func = dlsym( library, register_function_name.c_str() );
      );
      LOG_DEBUG("algorithm_plugin_manager::impl::register_from_module",
                "-> returned function address: " << register_func);

      GNUC_EXTENSION register_impls_func_t const register_impls
        = reinterpret_cast<register_impls_func_t>(register_func);

      // Check for symbol discovery
      if (!register_impls)
      {
        LOG_DEBUG("algorithm_plugin_manager::impl::register_from_module",
                  "-> Failed to find/load algorithm impl registration function");
      }
      // Call function, check for success
      else if ( (*register_impls)(registrar::instance()) > 0 )
      {
        LOG_WARN("algorithm_plugin_manager::impl::register_from_module",
                 "-> Algorithm implementation registration failed for one or " <<
                 "more algorithms in plugin module, possibly due to duplicate " <<
                 "registration: " << module_path);
        // TODO: Throw exception here?
      }
      else
      {
        LOG_DEBUG("algorithm_plugin_manager::impl::register_from_module",
                  "-> Successfully called registration func");
        module_used = true;

        // Adding module name to registered list
        registered_modules_[module_path.stem().string()] = module_path;
      }
    } // end Algorithm Implementation interface


    if (!module_used)
    {
      HANDLE_PLATFORM(
        /* Windows */
        if ( ! FreeLibrary(library) )
        {
          LOG_WARN("algorithm_plugin_manager::impl::register_from_module",
                   "Failed to free Windows module library: " << module_path);
        }
        ,
        /* Unix */
        if ( dlclose(library) )
        {
          LOG_WARN("algorithm_plugin_manager::impl::register_from_module",
                   "Failed to free Unix module library: " << module_path
                   << " (" << dlerror() << ")");
        }
      );
    }

    return module_used;
  }


  /// Get the list of registered modules names
  std::vector<std::string> registered_module_names() const
  {
    std::vector<std::string> r_vec;
    BOOST_FOREACH( registered_modules_t::value_type p, registered_modules_ )
    {
      r_vec.push_back( p.first );
    }
    return r_vec;
  }

};


// ===========================================================================
// PluginManager Implementation
// ---------------------------------------------------------------------------

/// Private constructor
algorithm_plugin_manager
::algorithm_plugin_manager()
  : impl_(new impl())
{
  // craft default search paths
  if( use_build_plugin_dir )
  {
    this->impl_->search_paths_.push_back( default_plugin_dir_build );
  }
  this->impl_->search_paths_.push_back( default_plugin_dir_install );

  if ( ! extra_path.empty() )
  {
    this->impl_->search_paths_.push_back( extra_path );
  }
}


/// Private destructor
algorithm_plugin_manager
::~algorithm_plugin_manager()
{
  delete this->impl_;
}


/// Access singleton instance of this class
algorithm_plugin_manager&
algorithm_plugin_manager
::instance()
{
  static algorithm_plugin_manager *instance_ = 0;
  if (!instance_)
  {
    instance_ = new algorithm_plugin_manager();
  }
  return *instance_;
}


/// (Re)Load plugin libraries found along current search paths
void
algorithm_plugin_manager
::register_plugins( std::string name )
{
# ifdef BUILD_SHARED_LIBS
  // If in dynamic mode, search for libraries to dlopen for algorithm
  // registration call-back.
  LOG_DEBUG("algorithm_plugin_manager::register_plugins",
            "Dynamically loading plugin impls");
  this->impl_->load_from_search_paths( name );
# else
  // In static mode, so call known registration functions of compiled-in
  // modules
  LOG_DEBUG("algorithm_plugin_manager::register_plugins",
            "Statically loading plugin impl");
  static_register_algorithms();
# endif
}


/// Add an additional directory to search for plugins in.
void
algorithm_plugin_manager
::add_search_path(path_t dirpath)
{
  this->impl_->search_paths_.push_back(dirpath);
}


/// Get the list currently registered module names.
std::vector< std::string >
algorithm_plugin_manager
::registered_module_names() const
{
  return this->impl_->registered_module_names();
}


} // end maptk namespace
