/*ckwg +5
 * Copyright 2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_ALGO_DETECT_FEATURES_H_
#define MAPTK_ALGO_DETECT_FEATURES_H_

#include <maptk/core/image.h>
#include <maptk/core/feature_set.h>

namespace maptk
{

namespace algo
{

/// An abstract base class for detecting feature points
class detect_features
{
public:
  /// Extract a set of image features from the provided image
  /// \param image_data contains the image data to process
  /// \returns a set of image features
  virtual feature_set_sptr
  detect(image_container_sptr image_data) const = 0;

};

} // end namespace algo

} // end namespace maptk


#endif // MAPTK_ALGO_DETECT_FEATURES_H_
