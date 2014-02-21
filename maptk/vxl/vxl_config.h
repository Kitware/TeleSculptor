/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief VXL configuration header
 */

#ifndef MAPTK_VXL_VXL_CONFIG_H
#define MAPTK_VXL_VXL_CONFIG_H

#include <maptk/config.h>

/// Define symbol visibility in maptk::vxl
#ifndef MAPTK_VXL_EXPORT
# ifdef MAKE_MAPTK_VXL_LIB
#   define MAPTK_VXL_EXPORT MAPTK_EXPORT
# else
#   define MAPTK_VXL_EXPORT MAPTK_IMPORT
# endif
# define MAPTK_VXL_NO_EXPORT MAPTK_NO_EXPORT
#endif

#endif
