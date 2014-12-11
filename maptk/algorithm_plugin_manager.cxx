
#include "algorithm_plugin_manager.h"

#include <string>

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


// CMake boolean convertion macros
#define ON true
#define OFF false


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
static std::string const library_suffix = std::string(LIBRARY_SUFFIX);

// Default module directory locations. Values defined in CMake configuration.
static maptk::path_t const default_plugin_dir_build = maptk::path_t(DEFAULT_PLUGIN_DIR_BUILD),
                           default_plugin_dir_install = maptk::path_t(DEFAULT_PLUGIN_DIR_INSTALL);
bool const use_build_plugin_dir = USE_BUILD_PLUGIN_DIR;

} // end anonymous namespace


// ===========================================================================
// PluginManager Private Implementation
// ---------------------------------------------------------------------------

class algorithm_plugin_manager::impl
{
// Memeber Variables ---------------------------------------------------------
public:
  std::vector<path_t> search_paths_;

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
      if ( boost::ends_with(e.path().string(), library_suffix)
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
      library = dlopen( module_path.string().c_str(), RTLD_LAZY );
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
        LOG_ERROR("algorithm_plugin_manager::impl::register_from_module",
                  "-> Algorithm implementation registration failed for one or " <<
                  "more algorithms in plugin module: " << module_path);
        // TODO: Throw exception here?
      }
      else
      {
        LOG_DEBUG("algorithm_plugin_manager::impl::register_from_module",
                  "-> Successfully called registration func");
        module_used = true;
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
  LOG_DEBUG("algorithm_plugin_manager::register_plugins", "Loading plugin impls");
  this->impl_->load_from_search_paths( name );
}


/// Add an additional directory to search for plugins in.
void
algorithm_plugin_manager
::add_search_path(path_t dirpath)
{
  this->impl_->search_paths_.push_back(dirpath);
}


} // end maptk namespace
