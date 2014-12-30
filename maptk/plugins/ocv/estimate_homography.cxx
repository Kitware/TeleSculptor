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
 * \brief OCV estimate_homography algorithm implementation
 */

#include "estimate_homography.h"

#include <boost/foreach.hpp>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>


namespace maptk
{

namespace ocv
{


/// Estimate a homography matrix from corresponding points
homography
estimate_homography
::estimate(const std::vector<vector_2d>& pts1,
           const std::vector<vector_2d>& pts2,
           std::vector<bool>& inliers,
           double inlier_scale) const
{
  if (pts1.size() < 4 || pts2.size() < 4)
  {
    std::cerr << "Not enough points to estimate a homography" <<std::endl;
    return homography::Zero();
  }

  std::vector<cv::Point2f> points1, points2;
  BOOST_FOREACH(const vector_2d& v, pts1)
  {
    points1.push_back(cv::Point2f(static_cast<float>(v.x()),
                                  static_cast<float>(v.y())));
  }
  BOOST_FOREACH(const vector_2d& v, pts2)
  {
    points2.push_back(cv::Point2f(static_cast<float>(v.x()),
                                  static_cast<float>(v.y())));
  }

  cv::Mat inliers_mat;
  cv::Mat H = cv::findHomography( cv::Mat(points1), cv::Mat(points2),
                                 CV_RANSAC,
                                 inlier_scale,
                                 inliers_mat );
  inliers.resize(inliers_mat.rows);
  for(unsigned i=0; i<inliers.size(); ++i)
  {
    inliers[i] = inliers_mat.at<bool>(i);
  }

  matrix_3x3d H_mat;
  cv2eigen(H, H_mat);
  return H_mat;
}


} // end namespace ocv

} // end namespace maptk
