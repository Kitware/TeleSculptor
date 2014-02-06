/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief Implementation of OCV DescriptorExtractor wrapping.
 */

#include <maptk/ocv/extract_descriptors.h>
#include <maptk/ocv/image_container.h>
#include <maptk/ocv/feature_set.h>
#include <maptk/ocv/descriptor_set.h>

#include <maptk/ocv/ocv_algo_tools.h>


namespace maptk
{

namespace ocv
{

/// Private implementation class
class extract_descriptors::priv
{
public:
  /// Constructor
  priv()
  : extractor(cv::DescriptorExtractor::create("SURF"))
  {
  }

  /// Copy constructor
  priv(const priv& other)
  : extractor(cv::DescriptorExtractor::create("SURF"))
  {
  }

  /// the descriptor extractor algorithm
  cv::Ptr<cv::DescriptorExtractor> extractor;
};


/// Constructor
extract_descriptors
::extract_descriptors()
: d_(new priv)
{
}


/// Copy Constructor
extract_descriptors
::extract_descriptors(const extract_descriptors& other)
: d_(new priv(*other.d_))
{
}


/// Destructor
extract_descriptors
::~extract_descriptors()
{
}


/// Get this algorithm's \link maptk::config_block configuration block \endlink
config_block_sptr
extract_descriptors
::get_configuration() const
{
  // base configuration block
  config_block_sptr config = algorithm::get_configuration();

  get_nested_ocv_algo_configuration("extractor", config, d_->extractor);

  return config;
}


/// Set this algorithm's properties via a config block
void
extract_descriptors
::set_configuration(config_block_sptr config)
{
  set_nested_ocv_algo_configuration(
      "extractor", config, d_->extractor);
}


/// Check that the algorithm's configuration config_block is valid
bool
extract_descriptors
::check_configuration(config_block_sptr config) const
{
  bool nested_ok = check_nested_ocv_algo_configuration(
      "extractor", config);

  return nested_ok;
}


/// Extract from the image a descriptor corresoponding to each feature
descriptor_set_sptr
extract_descriptors
::extract(image_container_sptr image_data,
          feature_set_sptr features) const
{
  if( !image_data || !features )
  {
    return descriptor_set_sptr();
  }
  cv::Mat img = image_container_to_ocv_matrix(*image_data);
  std::vector<cv::KeyPoint> kpts = features_to_ocv_keypoints(*features);

  cv::Mat desc;
  d_->extractor->compute( img, kpts, desc );

  return descriptor_set_sptr(new ocv::descriptor_set(desc));
}


} // end namespace ocv

} // end namespace maptk
