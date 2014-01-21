/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */


#include <maptk/ocv/estimate_homography.h>
#include <maptk/ocv/matrix.h>
#include <boost/foreach.hpp>
#include <opencv2/calib3d/calib3d.hpp>

namespace maptk
{

namespace ocv
{


/// Estimate a homography matrix from corresponding points
matrix_3x3d
estimate_homography
::estimate(const std::vector<vector_2d>& pts1,
           const std::vector<vector_2d>& pts2,
           std::vector<bool>& inliers,
           double inlier_scale) const
{
  if (pts1.size() < 4 || pts2.size() < 4)
  {
    std::cerr << "Not enough points to estimate a homography" <<std::endl;
    return matrix_3x3d(0.0);
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

  return matrix_from_ocv<3,3,double>(H);
}


} // end namespace ocv

} // end namespace maptk
