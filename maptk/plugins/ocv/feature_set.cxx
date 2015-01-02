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
 * \brief OCV feature_set implementation
 */

#include "feature_set.h"


namespace maptk
{

namespace ocv
{


/// Return a vector of feature shared pointers
std::vector<feature_sptr>
feature_set
::features() const
{
  typedef std::vector<cv::KeyPoint>::const_iterator cvKP_itr;
  std::vector<feature_sptr> features;
  for(cvKP_itr it = data_.begin(); it != data_.end(); ++it)
  {
    const cv::KeyPoint& kp = *it;
    feature_f *f = new feature_f();
    f->set_loc(vector_2f(kp.pt.x, kp.pt.y));
    f->set_magnitude(kp.response);
    f->set_scale(kp.size);
    f->set_angle(kp.angle);
    features.push_back(feature_sptr(f));
  }
  return features;
}


/// Convert any feature set to a vector of OpenCV cv::KeyPoints
std::vector<cv::KeyPoint>
features_to_ocv_keypoints(const maptk::feature_set& feat_set)
{
  if( const ocv::feature_set* f =
          dynamic_cast<const ocv::feature_set*>(&feat_set) )
  {
    return f->ocv_keypoints();
  }
  std::vector<cv::KeyPoint> kpts;
  std::vector<feature_sptr> feat = feat_set.features();
  typedef std::vector<feature_sptr>::const_iterator feat_itr;
  for(feat_itr it = feat.begin(); it != feat.end(); ++it)
  {
    const feature_sptr f = *it;
    cv::KeyPoint kp;
    vector_2d pt = f->loc();
    kp.pt.x = static_cast<float>(pt.x());
    kp.pt.y = static_cast<float>(pt.y());
    kp.response = static_cast<float>(f->magnitude());
    kp.size = static_cast<float>(f->scale());
    kp.angle = static_cast<float>(f->angle());
    kpts.push_back(kp);
  }
  return kpts;
}


} // end namespace ocv

} // end namespace maptk
