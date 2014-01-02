/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_PROJ_PROJ_CONFIG_H
#define MAPTK_PROJ_PROJ_CONFIG_H

#include <maptk/config.h>

/// Define symbol visibility in maptk::proj
#ifndef MAPTK_PROJ_EXPORT
# ifdef MAKE_MAPTK_PROJ_LIB
#   define MAPTK_PROJ_EXPORT MAPTK_EXPORT
# else
#   define MAPTK_PROJ_EXPORT MAPTK_IMPORT
# endif
# define MATPK_PROJ_NO_EXPORT MAPTK_NO_EXPORT
#endif

#endif
