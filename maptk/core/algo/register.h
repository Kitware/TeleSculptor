/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief core algorithm registration interface
 */

#ifndef MAPTK_ALGO_REGISTER_H_
#define MAPTK_ALGO_REGISTER_H_

#include <maptk/core/core_config.h>

namespace maptk
{

namespace algo
{

/// register all algorithms in this module
MAPTK_CORE_EXPORT void register_algorithms();


} // end namespace algo

} // end namespace maptk


#endif // MAPTK_ALGO_REGISTER_H_
