/*ckwg +29
 * Copyright 2015 by Kitware, Inc.
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
 * \brief Header for Ceres reprojection error functions
 */

#ifndef MAPTK_PLUGINS_CERES_REPROJECTION_ERROR_H_
#define MAPTK_PLUGINS_CERES_REPROJECTION_ERROR_H_


#include <maptk/plugins/ceres/ceres_config.h>

#include <ceres/ceres.h>
#include <ceres/rotation.h>


namespace maptk
{

namespace ceres
{

/// Ceres functor for perspective pinhole camera with no distortion
/**
 *Data parameter blocks are the following <2,5,6,3>
 *- 2 => residuals dimension,
 *- 5 => intrinsic data block [focal, principal point [x,y], aspect, skew],
 *- 6 => camera extrinsic data block (camera orientation and position) [R; c],
 *       - rotation(angle axis), and camera center [rX,rY,rZ,cx,cy,cz].
 *- 3 => a 3D point data block.
 *
 */
class reprojection_error
{
public:
  /// Constructor
  reprojection_error(const double x, const double y)
      : x_(x), y_(y) {}

  /// Indices of camera intrsic parameter block.
  enum {
    OFFSET_FOCAL_LENGTH = 0,
    OFFSET_PRINCIPAL_POINT_X = 1,
    OFFSET_PRINCIPAL_POINT_Y = 2,
    OFFSET_ASPECT_RATIO = 3,
    OFFSET_SKEW = 4,
  };

  /// Reprojection error functor for use in Ceres
  /**
   * \param [in] intrinsics: Camera intrinsics data block
   * \param [in] pose: Camera pose (extrinsics) data block
   *             - 3 for rotation(angle axis), 3 for center
   * \param [in] point: 3d point.
   * \param [out] residuals
   */
  template <typename T> bool operator()(const T* const intrinsics,
                                        const T* const pose,
                                        const T* const point,
                                        T* residuals) const {
    // Apply external parameters (Pose)
    const T* rotation = pose;
    const T* center = pose + 3;

    T translated_point[3];
    translated_point[0] = point[0] - center[0];
    translated_point[1] = point[1] - center[1];
    translated_point[2] = point[2] - center[2];

    T rotated_translated_point[3];
    // Rotate the point according the camera rotation
    ::ceres::AngleAxisRotatePoint(rotation,
                                  translated_point,
                                  rotated_translated_point);

    // Transform the point from homogeneous to euclidean
    const T x = rotated_translated_point[0] / rotated_translated_point[2];
    const T y = rotated_translated_point[1] / rotated_translated_point[2];

    // Apply intrinsic parameters
    const T& focal = intrinsics[OFFSET_FOCAL_LENGTH];
    const T& principal_point_x = intrinsics[OFFSET_PRINCIPAL_POINT_X];
    const T& principal_point_y = intrinsics[OFFSET_PRINCIPAL_POINT_Y];
    const T& aspect_ratio = intrinsics[OFFSET_ASPECT_RATIO];
    const T& skew = intrinsics[OFFSET_SKEW];

    // Apply instrinsics to get the final image coordinates
    const T projected_x = principal_point_x + focal * x + skew * y;
    const T projected_y = principal_point_y + focal / aspect_ratio * y;

    // Compute the reprojection error
    // difference between the predicted and observed position
    residuals[0] = projected_x - T(x_);
    residuals[1] = projected_y - T(y_);
    return true;
  }

  static ::ceres::CostFunction* create(const double x, const double y)
  {
    return new ::ceres::AutoDiffCostFunction<reprojection_error, 2, 5, 6, 3>(
        new reprojection_error(x, y));
  }

  double x_;
  double y_;
};


} // end namespace ceres

} // end namespace maptk


#endif // MAPTK_PLUGINS_CERES_REPROJECTION_ERROR_H_
