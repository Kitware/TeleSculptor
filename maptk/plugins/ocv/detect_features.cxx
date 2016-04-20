/*ckwg +29
 * Copyright 2013-2015 by Kitware, Inc.
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
 * \brief OCV detect_features algorithm implementation
 */

#include "detect_features.h"

#include <vector>

#include <vital/algo/algorithm.txx>
#include <vital/exceptions/image.h>

#include <maptk/plugins/ocv/feature_set.h>
#include <maptk/plugins/ocv/image_container.h>

using namespace kwiver::vital;

namespace kwiver {
namespace maptk {

namespace ocv
{


/// Extract a set of image features from the provided image
vital::feature_set_sptr
detect_features
::detect(vital::image_container_sptr image_data, vital::image_container_sptr mask) const
{
  cv::Mat cv_img = ocv::image_container::maptk_to_ocv(image_data->get_image());
  cv::Mat cv_mask;
  std::vector<cv::KeyPoint> keypoints;

  // Only initialize a mask image if the given mask image container contained
  // valid data.
  if( mask && mask->size() > 0 )
  {
    if ( image_data->width() != mask->width() ||
         image_data->height() != mask->height() )
    {
      throw image_size_mismatch_exception(
          "OCV detect feature algorithm given a non-zero mask with mismatched "
          "shape compared to input image",
          image_data->width(), image_data->height(),
          mask->width(), mask->height()
          );
    }

    // Make sure we make a one-channel cv::Mat
    vital::image s = mask->get_image();
    // hijacking memory of given mask image, but only telling the new image
    // object to consider the first channel. See vital::image documentation.
    vital::image i(s.memory(),
                     static_cast< vital::image::byte* >(s.memory()->data()),
                     s.width(),  s.height(), 1 /*depth*/,
                     s.w_step(), s.h_step(), s.d_step());
    cv_mask = ocv::image_container::maptk_to_ocv(i);
  }

  detector->detect(cv_img, keypoints, cv_mask);
  return feature_set_sptr(new feature_set(keypoints));
}


} // end namespace ocv

} // end namespace maptk
} // end namespace kwiver
