/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_VISCL_UTILS_H_

#include <maptk/core/feature_set.h>

namespace maptk
{

namespace vcl
{

/// Compute image dimensions from feature set
void min_image_dimensions(const maptk::feature_set &feat, unsigned int &width, unsigned int &height);

} // end namespace vcl

} // end namespace maptk

#endif // MAPTK_VISCL_UTILS_H_
