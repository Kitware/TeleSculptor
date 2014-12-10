
/**
 * \file
 * \brief Singleton manager of plug-in stuff
 */

#include <vector>

#include <boost/shared_ptr.hpp>

#include <maptk/config.h>
#include <maptk/types.h>


namespace maptk
{


/**
 * Plugin Manager for algorithm implementation extensions
 */
class MAPTK_LIB_EXPORT plugin_manager
{
public:
  /// Get the reference to the singleton instance of this class
  static plugin_manager& instance();

  /// (Re)Load plugin libraries found along current search paths
  void register_plugins();

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
  plugin_manager();
  /// private deconstructor (singleton)
  virtual ~plugin_manager();
  /// private copy constructor (singleton)
  plugin_manager(plugin_manager const&);
  /// private assignment operator (singleton)
  plugin_manager& operator=(plugin_manager const&);
};


} // end maptk namespace
