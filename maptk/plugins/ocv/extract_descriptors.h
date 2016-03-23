/*ckwg +29
 * Copyright 2013-2016 by Kitware, Inc.
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
 * \brief Extended algorithm definition for OpenCV descriptor extractor algos
 */

#ifndef MAPTK_PLUGINS_OCV_EXTRACT_DESCRIPTORS_H_
#define MAPTK_PLUGINS_OCV_EXTRACT_DESCRIPTORS_H_

#include <vital/vital_config.h>
#include <vital/algo/extract_descriptors.h>

#include <maptk/plugins/ocv/maptk_ocv_export.h>

#include <opencv2/features2d/features2d.hpp>

namespace kwiver {
namespace maptk {
namespace ocv {

/// OCV specific definition for algorithms that describe feature points
/**
 * This extended algorithm_def provides a common implementation for the extract
 * method.
 */
class MAPTK_OCV_EXPORT extract_descriptors
  : public vital::algo::extract_descriptors
{
public:
  /// Extract from the image a descriptor corresponding to each feature
  /**
   * \param image_data contains the image data to process
   * \param features the feature locations at which descriptors are extracted
   * \returns a set of feature descriptors
   */
  virtual vital::descriptor_set_sptr
  extract(vital::image_container_sptr image_data,
          vital::feature_set_sptr features,
          vital::image_container_sptr image_mask = vital::image_container_sptr()) const;

protected:
  /// the descriptor extractor algorithm
  cv::Ptr<cv::DescriptorExtractor> extractor;
};

} // end namespace ocv
} // end namespace maptk
} // end namespace kwiver


#endif // MAPTK_PLUGINS_OCV_EXTRACT_DESCRIPTORS_H_
