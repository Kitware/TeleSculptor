/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include <maptk/viscl/register.h>
#include <maptk/viscl/detect_features.h>
#include <maptk/viscl/extract_descriptors.h>
#include <maptk/viscl/match_features.h>

namespace maptk
{

namespace vcl
{

/// register all algorithms in this module
void register_algorithms()
{
  vcl::detect_features::register_self();
  vcl::extract_descriptors::register_self();
  vcl::match_features::register_self();
}


} // end namespace viscl

} // end namespace maptk
