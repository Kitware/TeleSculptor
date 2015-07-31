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
 * \brief OCV match_set interface
 */

#ifndef MAPTK_PLUGINS_OCV_MATCH_SET_H_
#define MAPTK_PLUGINS_OCV_MATCH_SET_H_

#include <opencv2/features2d/features2d.hpp>

#include <vital/types/match_set.h>

#include <maptk/plugins/ocv/ocv_config.h>


namespace kwiver {
namespace maptk {

namespace ocv
{


/// A concrete match set that wraps OpenCV cv::DMatch objects
class MAPTK_OCV_EXPORT match_set
  : public kwiver::vital::match_set
{
public:
  /// Default constructor
  match_set() {}

  /// Constructor from a vector of cv::DMatch
  explicit match_set(const std::vector<cv::DMatch>& matches)
  : data_(matches) {}

  /// Return the number of matches in the set
  virtual size_t size() const { return data_.size(); }

  /// Return a vector of matching indices
  virtual std::vector<kwiver::vital::match> matches() const;

  /// Return the underlying OpenCV match data structures
  const std::vector<cv::DMatch>& ocv_matches() const { return data_; }

private:
  // The vector of OpenCV match structures
  std::vector<cv::DMatch> data_;
};


/// Convert any match set to a vector of OpenCV cv::DMatch
MAPTK_OCV_EXPORT std::vector<cv::DMatch>
matches_to_ocv_dmatch(const kwiver::vital::match_set& match_set);


} // end namespace ocv

} // end namespace maptk
} // end namespace kwiver


#endif // MAPTK_PLUGINS_OCV_MATCH_SET_H_
