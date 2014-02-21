/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief core module configuration header
 */

#ifndef MAPTK_CORE_CORE_CONFIG_H
#define MAPTK_CORE_CORE_CONFIG_H

#include <maptk/config.h>

/// Define symbol visibility in maptk::core
#ifndef MAPTK_CORE_EXPORT
# ifdef MAKE_MAPTK_CORE_LIB
#   define MAPTK_CORE_EXPORT MAPTK_EXPORT
# else
#   define MAPTK_CORE_EXPORT MAPTK_IMPORT
# endif
/// Mark labeled symbols as not to be exported
# define MAPTK_CORE_NO_EXPORT MAPTK_NO_EXPORT
#endif

#endif
