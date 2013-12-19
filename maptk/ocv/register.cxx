/*ckwg +5
 * Copyright 2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include <maptk/ocv/register.h>
#include <maptk/ocv/image_io.h>
#include <maptk/ocv/detect_features.h>

namespace maptk
{

namespace ocv
{

/// register all algorithms in this module
void register_algorithms()
{
  ocv::image_io::register_self();
  ocv::detect_features::register_self();
}


} // end namespace ocv

} // end namespace maptk
