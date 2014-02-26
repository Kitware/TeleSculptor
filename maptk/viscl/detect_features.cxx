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
  priv() : max_kpts(5000), thresh(0.003f), sigma(2.0f)
  {
  }

  /// Copy Constructor
  priv(const priv& other)
  {
  }

  viscl::hessian detector;
  unsigned int max_kpts;
  float thresh;
  float sigma;
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

/// Get this algorithm's \link maptk::config_block configuration block \endlink
config_block_sptr
detect_features
::get_configuration() const
{
  config_block_sptr config = algorithm::get_configuration();
  config->set_value("max_keypoints", "5000", "Maximum number of features to detect on an image.");
  config->set_value("thresh", "0.003", "Threshold on the determinant of Hessian for keypoint candidates.");
  config->set_value("sigma", "2.0", "Smoothing scale.");
  return config;
}

/// Set this algorithm's properties via a config block
void
detect_features
::set_configuration(config_block_sptr config)
{
  d_->max_kpts = config->get_value<unsigned int>("max_keypoints", 5000);
  d_->thresh = config->get_value<float>("thresh", 0.003f);
  d_->sigma = config->get_value<float>("sigma", 2.0f);
}

/// Check that the algorithm's configuration config_block is valid
bool
detect_features
::check_configuration(config_block_sptr config) const
{
  return true;
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

  d_->detector.smooth_and_detect(img, feature_data.kptmap_, feature_data.features_, feature_data.numfeat_,
                                 d_->max_kpts, d_->thresh, d_->sigma);

  return feature_set_sptr(new feature_set(feature_data));
}


} // end namespace vcl

} // end namespace maptk
