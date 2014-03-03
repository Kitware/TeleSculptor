/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include "utils.h"

namespace maptk
{

namespace vcl
{

/// Compute image dimensions from feature set
void min_image_dimensions(const maptk::feature_set &feat, unsigned int &width, unsigned int &height)
{
  width = 0;
  height = 0;

  std::vector<feature_sptr> features = feat.features();
  for (unsigned int i = 0; i < features.size(); i++)
  {
    if (width < features[i]->loc()[0])
      width = features[i]->loc()[0];
    if (height < features[i]->loc()[1])
      height = features[i]->loc()[1];
  }
}

} // end namespace vcl

} // end namespace maptk
