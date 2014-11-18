
#ifndef _MAPTK_PLUGIN_DEFAULT_CONFIG_H_
#define _MAPTK_PLUGIN_DEFAULT_CONFIG_H_

#include <maptk/config.h>


/// Toggle symbol export syntax when building plugin library
#ifndef PLUGIN_DEFAULT_EXPORT
# ifdef MAKE_PLUGIN_DEFAULT
#   define PLUGIN_DEFAULT_EXPORT MAPTK_EXPORT
# else
#   define PLUGIN_DEFAULT_EXPORT
# endif
#endif


#endif // _MAPTK_PLUGIN_DEFAULT_CONFIG_H_
