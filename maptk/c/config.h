
#ifndef MAPTK_C_CONFIG_H_
#define MAPTK_C_CONFIG_H_

#include <maptk/config.h>


#ifndef MAPTK_C_EXPORT
# ifdef MAKE_MAPTK_C_LIB
#   define MAPTK_C_EXPORT MAPTK_EXPORT
# else
#   define MAPTK_C_EXPORT MAPTK_IMPORT
# endif
#endif


#endif // MAPTK_C_CONFIG_H_
