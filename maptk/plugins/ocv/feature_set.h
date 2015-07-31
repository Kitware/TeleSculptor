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
 * \brief OCV feature_set interface
 */

#ifndef MAPTK_PLUGINS_OCV_FEATURE_SET_H_
#define MAPTK_PLUGINS_OCV_FEATURE_SET_H_

#include <opencv2/features2d/features2d.hpp>

#include <vital/types/feature_set.h>

#include <maptk/plugins/ocv/ocv_config.h>


namespace kwiver {
namespace maptk {

namespace ocv
{


/// A concrete feature set that wraps OpenCV KeyPoints
class MAPTK_OCV_EXPORT feature_set
  : public kwiver::vital::feature_set
{
public:
  /// Default Constructor
  feature_set() {}

  /// Constructor from a vector of cv::KeyPoints
  explicit feature_set(const std::vector<cv::KeyPoint>& features)
  : data_(features) {}

  /// Return the number of feature in the set
  virtual size_t size() const { return data_.size(); }

  /// Return a vector of feature shared pointers
  virtual std::vector<kwiver::vital::feature_sptr> features() const;

  /// Return the underlying OpenCV vector of cv::KeyPoints
  const std::vector<cv::KeyPoint>& ocv_keypoints() const { return data_; }

protected:

  /// The vector of KeyPoints
  std::vector<cv::KeyPoint> data_;
};


/// Convert any feature set to a vector of OpenCV cv::KeyPoints
MAPTK_OCV_EXPORT std::vector<cv::KeyPoint>
features_to_ocv_keypoints(const kwiver::vital::feature_set& features);


} // end namespace ocv

} // end namespace maptk
} // end namespace kwiver


#endif // MAPTK_PLUGINS_OCV_FEATURE_SET_H_
