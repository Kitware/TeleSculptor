/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_OCV_REGISTER_H_
#define MAPTK_OCV_REGISTER_H_

#include "ocv_config.h"

namespace maptk
{

namespace ocv
{

/// register all algorithms in this module
MAPTK_OCV_EXPORT void register_algorithms();


} // end namespace ocv

} // end namespace maptk


#endif // MAPTK_OCV_REGISTER_H_
