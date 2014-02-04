/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include "detect_features.h"

#include <vector>
#include <maptk/viscl/feature_set.h>
#include <maptk/viscl/image_container.h>

#include <viscl/tasks/hessian.h>


namespace maptk
{

namespace vcl
{


/// Private implementation class
class detect_features::priv
{
public:
  /// Constructor
  priv()
  {
  }

  /// Copy Constructor
  priv(const priv& other)
  {
  }

  viscl::hessian detector;
};


/// Constructor
detect_features
::detect_features()
: d_(new priv)
{
}


/// Copy Constructor
detect_features
::detect_features(const detect_features& other)
: d_(new priv(*other.d_))
{
}


/// Destructor
detect_features
::~detect_features()
{
}


/// Extract a set of image features from the provided image
/// \param image_data contains the image data to process
/// \returns a set of image features
feature_set_sptr
detect_features
::detect(image_container_sptr image_data) const
{
  viscl::image img = vcl::image_container_to_viscl(*image_data);
  vcl::feature_set::type feature_data;

  //TODO: add config for smoothing sigma, max kpts, and threshold
  const unsigned int max_kpts = 5000;
  const float thresh = 0.03f;
  const float sigma = 2.0f;

  d_->detector.smooth_and_detect(img, feature_data.kptmap_, feature_data.features_, feature_data.numfeat_, max_kpts, thresh, sigma);

  return feature_set_sptr(new feature_set(feature_data));
}


} // end namespace viscl

} // end namespace maptk
