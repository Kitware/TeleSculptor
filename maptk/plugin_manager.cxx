
#include "plugin_manager.h"

#include <string>

#if defined(_WIN32) || defined(_WIN64)
# include <windows.h>
#else
# include <dlfcn.h>
#endif

#include <boost/algorithm/string/predicate.hpp>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>

#include <maptk/exceptions/io.h>
#include <maptk/logging_macros.h>


namespace bfs = boost::filesystem;


namespace maptk
{


namespace // anonymous
{


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
typedef int (*register_impls_func_t)(void);


static std::string const register_function_name = std::string("register_algo_impls");
// Platform specific plugin library file (set as compile definition in CMake)
static std::string const library_suffix = std::string(LIBRARY_SUFFIX);


}


class plugin_manager::impl
{
public:
  impl()
    : search_paths_()
  {}

  std::vector<path_t> search_paths_;

  /// Attempt loading algorithm implementations from all known search paths
  void load_from_search_paths()
  {
    LOG_DEBUG("plugin_manager::impl::load_from_search_paths",
              "Loading plugins in search paths");
    BOOST_FOREACH( path_t module_dir, this->search_paths_ )
    {
      // TODO: Probably going to have to do something here in regards to
      //       windows and build configuration subdirectories
      load_modules_in_directory(module_dir);
    }
  }

  /// Attempt loading algorithm implementations from all plugin modules in dir
  void load_modules_in_directory(path_t dir_path)
  {
    // Preventing load from current directory (security)
    if (dir_path.empty())
    {
      return;
    }
    if (!bfs::exists(dir_path))
    {
      throw path_not_exists(dir_path);
    }
    if (!bfs::is_directory(dir_path))
    {
      throw path_not_a_directory(dir_path);
    }

    LOG_DEBUG("plugin_manager::impl::load_modules_in_directory",
              "Loading modules in directory: " << dir_path);
    bfs::directory_iterator dir_it(dir_path);
    while (dir_it != bfs::directory_iterator())
    {
      bfs::directory_entry const e = *dir_it;

      LOG_DEBUG("plugin_manager::impl::load_modules_in_directory",
                "Testing that file " << e.path() << " ends with " <<
                library_suffix);
      if (boost::ends_with(e.path().string(), library_suffix))
      {
        // Check that we're looking a file
        if (e.status().type() == bfs::regular_file)
        {
          register_from_module(e.path());
        }
        else
        {
          LOG_WARN("plugin_manager::impl::load_modules_in_directory",
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
   * \returns True of the module was able to be loaded. False if the module
   *          could not be loaded, or after successfully finding and loading
   *          the function, but the function returns a failure.
   *
   * TODO: Just use exception handing instead of return codes and booleans.
   */
  bool register_from_module(path_t module_path)
  {
    LOG_DEBUG("plugin_manager::impl::register_from_module",
              "Starting algorithm implementation registration for plugin "
              "module file: " << module_path);

    // Load module library, returning false if it couldn't be loaded
    library_t library = NULL;
    std::string err_str;
    HANDLE_PLATFORM(
      /* windows */
      library = LoadLibrary( module_path.string().c_str() );
      // TODO: Catch platform specific error string on failure
      ,
      /* unix */
      dlerror();
      library = dlopen( module_path.string().c_str(), RTLD_LAZY );
      if (!library)
      {
        err_str = std::string(dlerror());
      }
    );

    if (!library)
    {
      LOG_ERROR("plugin_manager::impl::register_from_module",
                "Failed to open module library " << module_path <<
                " (" << err_str << ")");
      return false;
    }
    LOG_DEBUG("plugin_manager::impl::register_from_module",
              "Loaded module: " << library);

    // Attempt to load assumed registration function. If function not found, we
    // assume this plugin doesn't provide any algorithm implementations and
    // close the library. We otherwise keep it open if we are going to use
    // things from it.
    LOG_DEBUG("plugin_manager::impl::register_from_module",
              "Looking for symbol: " << register_function_name.c_str());
    function_t register_func = NULL;
    HANDLE_PLATFORM(
      /* Windows */
      register_func = GetProcAddress( library, register_function_name.c_str() );
      ,
      /* Unix */
      register_func = dlsym( library, register_function_name.c_str() );
    );
    LOG_DEBUG("plugin_manager::impl::register_from_module",
              "register_func: " << register_func);

    GNUC_EXTENSION register_impls_func_t const register_impls
      = reinterpret_cast<register_impls_func_t>(register_func);

    bool unload_library = false,
         ret = true;
    if (!register_impls)
    {
      LOG_DEBUG("plugin_manager::impl::register_from_module",
                "Failed to find/load registration func");
      unload_library = true;
    }
    else if ( (*register_impls)() > 0 )
    {
      LOG_ERROR("plugin_manager::impl::register_from_module",
                "Algorithm implementation registration failed for one or " <<
                "more algorithms in plugin module: " << module_path);
      ret = false;
      unload_library = true;
    }
    else
    {
      LOG_DEBUG("plugin_manager::impl::register_from_module",
                "Successfully called registration func");
    }

    if (unload_library)
    {
      HANDLE_PLATFORM(
        /* Windows */
        if ( ! FreeLibrary(library) )
        {
          LOG_WARN("plugin_manager::impl::register_from_module",
                   "Failed to free Windows module library: " << module_path);
        }
        ,
        /* Unix */
        if ( dlclose(library) )
        {
          LOG_WARN("plugin_manager::impl::register_from_module",
                   "Failed to free Unix module library: " << module_path
                   << " (" << dlerror() << ")");
        }
      );
    }

    return ret;
  }

};


/// Access singleton instance of this class
plugin_manager&
plugin_manager
::instance()
{
  static plugin_manager *instance_ = 0;
  if (!instance_)
  {
    instance_ = new plugin_manager();
  }
  return *instance_;
}


/// (Re)Load plugin libraries found along current search paths
void
plugin_manager
::register_plugins()
{
  LOG_DEBUG("plugin_manager::register_plugins", "Loading plugin impls");
  this->impl_->load_from_search_paths();
}


/// Add an additional directory to search for plugins in.
void
plugin_manager
::add_search_path(path_t dirpath)
{
  this->impl_->search_paths_.push_back(dirpath);
}


plugin_manager
::plugin_manager()
  : impl_(new impl())
{
  this->impl_->search_paths_.push_back(MAPTK_DEFAULT_PLUGIN_DIR);
}


plugin_manager
::~plugin_manager()
{
  delete this->impl_;
}


} // end maptk namespace
