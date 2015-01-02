/*ckwg +29
 * Copyright 2013-2014 by Kitware, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither name of Kitware, Inc. nor the names of any contributors may be used
 *    to endorse or promote products derived from this software without specific
 *    prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHORS OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * \file
 * \brief Implementation of OCV DescriptorExtractor wrapping.
 */

#include "extract_descriptors.h"

#include <maptk/plugins/ocv/image_container.h>
#include <maptk/plugins/ocv/feature_set.h>
#include <maptk/plugins/ocv/descriptor_set.h>
#include <maptk/plugins/ocv/ocv_algo_tools.h>


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
  : extractor(default_extractor())
  {
  }

  /// Copy constructor
  priv(const priv& other)
  : extractor(default_extractor())
  {
  }

  /// create the default descriptor extractor
  static cv::Ptr<cv::DescriptorExtractor> default_extractor()
  {
    cv::Ptr<cv::DescriptorExtractor> ext;
    // try the SURF detector first
    ext = cv::DescriptorExtractor::create("SURF");
    if( !ext )
    {
      // if SURF is not available (nonfree not built) use ORB
      ext = cv::DescriptorExtractor::create("ORB");
    }
    return ext;
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
  bool nested_ok =
    check_nested_ocv_algo_configuration<cv::DescriptorExtractor>(
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
