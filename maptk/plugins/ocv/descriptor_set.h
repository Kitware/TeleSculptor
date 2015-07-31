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
 * \brief OCV descriptor_set interface
 */

#ifndef MAPTK_PLUGINS_OCV_DESCRIPTOR_SET_H_
#define MAPTK_PLUGINS_OCV_DESCRIPTOR_SET_H_

#include <vital/types/descriptor_set.h>

#include <maptk/plugins/ocv/ocv_config.h>

#include <opencv2/features2d/features2d.hpp>


namespace kwiver {
namespace maptk {

namespace ocv
{

/// A concrete descriptor set that wraps OpenCV descriptors.
class MAPTK_OCV_EXPORT descriptor_set
  : public kwiver::vital::descriptor_set
{
public:
  /// Default Constructor
  descriptor_set() {}

  /// Constructor from an OpenCV descriptor matrix
  explicit descriptor_set(const cv::Mat& descriptor_matrix)
  : data_(descriptor_matrix) {}

  /// Return the number of descriptor in the set
  virtual size_t size() const { return data_.rows; }

  /// Return a vector of descriptor shared pointers
  virtual std::vector<kwiver::vital::descriptor_sptr> descriptors() const;

  /// Return the native OpenCV descriptors as a matrix
  const cv::Mat& ocv_desc_matrix() const { return data_; }

protected:

  /// The OpenCV matrix of featrues
  cv::Mat data_;
};


/// Convert any descriptor set to an OpenCV cv::Mat
/**
 * \param desc_set descriptors to convert to cv::mat
 */
MAPTK_OCV_EXPORT cv::Mat
descriptors_to_ocv_matrix(const kwiver::vital::descriptor_set& desc_set);


} // end namespace ocv

} // end namespace maptk
} // end namespace kwiver


#endif // MAPTK_PLUGINS_OCV_DESCRIPTOR_SET_H_
