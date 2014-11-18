
#ifndef _MAPTK_PLUGIN_INTERFACE_ALGORITHM_IMPL_PL_INTERFACE_H_
#define _MAPTK_PLUGIN_INTERFACE_ALGORITHM_IMPL_PL_INTERFACE_H_

#include <maptk/registrar.h>


#ifdef __cplusplus
extern "C"
{
#endif


/// Register algorithm implementations in this function
/**
 * Implementations of this method within a plugin should include a
 * ``.register_self()`` call for each algorithm implementation to be made
 * available.
 *
 * \returns The number of implementations that FAILED to register, i.e. a
 *          return of 0 means registration success.
 *
 * Remember to also use 'extern "C" ...' in plugin implementation if compiling
 * in C++ to prevent the dreaded symbol name-mangling.
 */
int register_algo_impls(maptk::registrar &);


#ifdef __cplusplus
}
#endif


#endif // _MAPTK_PLUGIN_INTERFACE_ALGORITHM_IMPL_PL_INTERFACE_H_
