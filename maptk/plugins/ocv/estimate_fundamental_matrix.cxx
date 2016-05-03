/*ckwg +29
 * Copyright 2016 by Kitware, Inc.
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
 * \brief OCV estimate_fundamental_matrix algorithm implementation
 */

#include "estimate_fundamental_matrix.h"

#include <vital/vital_foreach.h>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

namespace kwiver {
namespace maptk {

namespace ocv
{

/// Private implementation class
class estimate_fundamental_matrix::priv
{
public:
  /// Constructor
  priv()
    : confidence_threshold(0.99)
  {
  }

  priv(const priv& other)
    : confidence_threshold(other.confidence_threshold)
  {
  }

  double confidence_threshold;
};


/// Constructor
estimate_fundamental_matrix
::estimate_fundamental_matrix()
: d_(new priv)
{
}


/// Copy Constructor
estimate_fundamental_matrix
::estimate_fundamental_matrix(const estimate_fundamental_matrix& other)
: d_(new priv(*other.d_))
{
}


/// Destructor
estimate_fundamental_matrix
::~estimate_fundamental_matrix()
{
}

/// Get this algorithm's \link vital::config_block configuration block \endlink
vital::config_block_sptr
estimate_fundamental_matrix
::get_configuration() const
{
  // get base config from base class
  vital::config_block_sptr config =
      vital::algo::estimate_fundamental_matrix::get_configuration();

  config->set_value("confidence_threshold", d_->confidence_threshold,
                    "Confidence that estimated matrix is correct, range (0.0, 1.0]");

  return config;
}


/// Set this algorithm's properties via a config block
void
estimate_fundamental_matrix
::set_configuration(vital::config_block_sptr config)
{
  d_->confidence_threshold = config->get_value<double>("confidence_threshold", d_->confidence_threshold);
}


/// Check that the algorithm's configuration vital::config_block is valid
bool
estimate_fundamental_matrix
::check_configuration(vital::config_block_sptr config) const
{
  double confidence_threshold = config->get_value<double>("confidence_threshold", d_->confidence_threshold);
  if( confidence_threshold <= 0.0 || confidence_threshold > 1.0 )
  {
    std::cerr << "confidence_threshold parameter is " << confidence_threshold << ", needs to be in (0.0, 1.0]." << std::endl;
    return false;
  }

  return true;
}


/// Estimate a fundamental matrix from corresponding points
vital::fundamental_matrix_sptr
estimate_fundamental_matrix
::estimate(const std::vector<vital::vector_2d>& pts1,
           const std::vector<vital::vector_2d>& pts2,
           std::vector<bool>& inliers,
           double inlier_scale) const
{
  if (pts1.size() < 8 || pts2.size() < 8)
  {
    std::cerr << "Not enough points to estimate a fundamental matrix" <<std::endl;
    return vital::fundamental_matrix_sptr();
  }

  std::vector<cv::Point2f> points1, points2;
  VITAL_FOREACH(const vital::vector_2d& v, pts1)
  {
    points1.push_back(cv::Point2f(static_cast<float>(v.x()),
                                  static_cast<float>(v.y())));
  }
  VITAL_FOREACH(const vital::vector_2d& v, pts2)
  {
    points2.push_back(cv::Point2f(static_cast<float>(v.x()),
                                  static_cast<float>(v.y())));
  }

  cv::Mat inliers_mat;
  cv::Mat F = cv::findFundamentalMat( cv::Mat(points1), cv::Mat(points2),
                                      CV_FM_RANSAC,
                                      inlier_scale,
                                      d_->confidence_threshold,
                                      inliers_mat );
  inliers.resize(inliers_mat.rows);
  for(unsigned i = 0; i < inliers.size(); ++i)
  {
    inliers[i] = inliers_mat.at<bool>(i);
  }

  vital::matrix_3x3d F_mat;
  cv2eigen(F, F_mat);
  return vital::fundamental_matrix_sptr( new vital::fundamental_matrix_<double>(F_mat) );
}


} // end namespace ocv

} // end namespace maptk
} // end namespace kwiver
