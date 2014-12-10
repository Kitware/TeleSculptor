
/**
 * \file
 * \brief Singleton manager of plug-in stuff
 */

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
