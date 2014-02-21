/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief VXL algorithm registration implementation
 */

#include <maptk/vxl/register.h>

#include <maptk/vxl/bundle_adjust.h>
#include <maptk/vxl/estimate_homography.h>
#include <maptk/vxl/image_io.h>

namespace maptk
{

namespace vxl
{

/// register all algorithms in this module
void register_algorithms()
{
  vxl::bundle_adjust::register_self();
  vxl::estimate_homography::register_self();
  vxl::image_io::register_self();
}


} // end namespace vxl

} // end namespace maptk
