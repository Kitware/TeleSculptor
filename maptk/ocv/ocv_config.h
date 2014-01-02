/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_OCV_OCV_CONFIG_H
#define MAPTK_OCV_OCV_CONFIG_H

#include <maptk/config.h>

/// Define symbol visibility in maptk::ocv
#ifndef MAPTK_OCV_EXPORT
# ifdef MAKE_MAPTK_OCV_LIB
#   define MAPTK_OCV_EXPORT MAPTK_EXPORT
# else
#   define MAPTK_OCV_EXPORT MAPTK_IMPORT
# endif
# define MAPTK_OCV_NO_EXPORT MAPTK_NO_EXPORT
#endif

#endif
