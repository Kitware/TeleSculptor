/*ckwg +29
 * Copyright 2015-2016 by Kitware, Inc.
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


#include <vital/vital_config.h>
#include <maptk/plugins/ceres/maptk_ceres_export.h>

#include <maptk/plugins/ceres/lens_distortion.h>
#include <maptk/plugins/ceres/types.h>

#include <ceres/ceres.h>
#include <ceres/rotation.h>


namespace kwiver {
namespace maptk {

namespace ceres
{


/// Helper function to apply pose transformations and project a point
/**
 * \param [in] pose: Camera pose (extrinsics) data block
 *             - 3 for rotation(angle axis), 3 for center
 * \param [in] point: 3D point.
 * \param [out] xy: projected 2D normalized image coordinate
 */
template <typename T>
void project_point(const T* const pose,
                   const T* const point,
                   T* xy)
{
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
  xy[0] = rotated_translated_point[0] / rotated_translated_point[2];
  xy[1] = rotated_translated_point[1] / rotated_translated_point[2];
}


/// Helper function to apply the intrinsic camera calibration matrix
/**
 * \param [in] intrinsics: [focal, principal point [x,y], aspect, skew]
 * \param [in] xy: 2D point in normalized image coordinates
 * \param [out] image_xy: 2D point in actual image coordinates
 */
template <typename T>
void apply_intrinsic_matrix(const T* intrinsics,
                            const T* xy,
                            T* image_xy)
{
  const T& x = xy[0];
  const T& y = xy[1];

  // Apply intrinsic parameters
  const T& focal = intrinsics[0];
  const T& principal_point_x = intrinsics[1];
  const T& principal_point_y = intrinsics[2];
  const T& aspect_ratio = intrinsics[3];
  const T& skew = intrinsics[4];

  // Apply instrinsics to get the final image coordinates
  image_xy[0] = principal_point_x + focal * x + skew * y;
  image_xy[1] = principal_point_y + focal / aspect_ratio * y;
}


/// Ceres reprojection error (RPE) functor with no lens distortion
/**
 *Data parameter blocks are the following <2,5,6,3>
 *- 2 => residuals dimension,
 *- 5 => intrinsic data block [focal, principal point [x,y], aspect, skew],
 *- 6 => camera extrinsic data block (camera orientation and position) [R; c],
 *       - rotation(angle axis), and camera center [rX,rY,rZ,cx,cy,cz].
 *- 3 => a 3D point data block.
 *
 */
class rpe_no_distortion
{
public:
  /// Constructor
  rpe_no_distortion(const double x, const double y)
      : x_(x), y_(y) {}

  /// Reprojection error functor for use in Ceres
  /**
   * \param [in] intrinsics: Camera intrinsics data block
   * \param [in] pose: Camera pose (extrinsics) data block
   *             - 3 for rotation(angle axis), 3 for center
   * \param [in] point: 3D point.
   * \param [out] residuals
   */
  template <typename T> bool operator()(const T* const intrinsics,
                                        const T* const pose,
                                        const T* const point,
                                        T* residuals) const
  {
    T xy[2], image_xy[2];

    // Project the point into 2D
    project_point(pose, point, xy);

    // Apply the intrinsic calibration matrix
    apply_intrinsic_matrix(intrinsics, xy, image_xy);

    // Compute the reprojection error
    // difference between the predicted and observed position
    residuals[0] = image_xy[0] - T(x_);
    residuals[1] = image_xy[1] - T(y_);
    return true;
  }

  /// Cost function factory
  static ::ceres::CostFunction* create(const double x, const double y)
  {
    typedef rpe_no_distortion Self;
    return new ::ceres::AutoDiffCostFunction<Self, 2, 5, 6, 3>(new Self(x, y));
  }

  double x_;
  double y_;
};


/// Ceres reprojection error (RPE) functor templated over distortion type
/**
 * The template parameter should be a struct containing
 *  - num_coeffs a static const int defining the number of distortion coefficients
 *  - apply a distortion function of with signature \code
 *        void (*)(const T* in_point, const T* coeffs, T* out_point);
 *    \endcode
 * Data parameter blocks are the following <2,5+ndp,6,3>
 *  - 2 => residuals dimension,
 *  - 5+ndp => intrinsic data block [focal, principal point [x,y], aspect, skew, d],
 *         - d contains ndp distortion parameters
 *  - 6 => camera extrinsic data block (camera orientation and position) [R; c],
 *         - rotation(angle axis), and camera center [rX,rY,rZ,cx,cy,cz].
 *  - 3 => a 3D point data block.
 *
 */
template <typename DF>
class rpe_distortion
{
public:
  /// Constructor
  rpe_distortion(const double x, const double y)
      : x_(x), y_(y) {}

  /// Reprojection error functor for use in Ceres
  /**
   * \param [in] intrinsics: Camera intrinsics data block
   * \param [in] pose: Camera pose (extrinsics) data block
   *             - 3 for rotation(angle axis), 3 for center
   * \param [in] point: 3D point.
   * \param [out] residuals
   */
  template <typename T> bool operator()(const T* const intrinsics,
                                        const T* const pose,
                                        const T* const point,
                                        T* residuals) const
  {
    T xy[2], distorted_xy[2], image_xy[2];

    // Project the point into 2D
    project_point(pose, point, xy);

    // Apply radial distotion
    DF::apply(intrinsics+5, xy, distorted_xy);

    // Apply the intrinsic calibration matrix
    apply_intrinsic_matrix(intrinsics, distorted_xy, image_xy);

    // Compute the reprojection error
    // difference between the predicted and observed position
    residuals[0] = image_xy[0] - T(x_);
    residuals[1] = image_xy[1] - T(y_);
    return true;
  }

  /// Cost function factory
  static ::ceres::CostFunction* create(const double x, const double y)
  {
    typedef rpe_distortion<DF> Self;
    // number of intrinsic parameters
    static const int nip = 5 + DF::num_coeffs;
    return new ::ceres::AutoDiffCostFunction<Self, 2, nip, 6, 3>(new Self(x, y));
  }

  double x_;
  double y_;
};


/// Factory to create Ceres cost functions for each lens distortion type
::ceres::CostFunction*
create_cost_func(LensDistortionType ldt, double x, double y)
{
  switch(ldt)
  {
  case POLYNOMIAL_RADIAL_DISTORTION:
    return rpe_distortion<distortion_poly_radial>::create(x,y);
  case POLYNOMIAL_RADIAL_TANGENTIAL_DISTORTION:
    return rpe_distortion<distortion_poly_radial_tangential>::create(x,y);
  case RATIONAL_RADIAL_TANGENTIAL_DISTORTION:
    return rpe_distortion<distortion_ratpoly_radial_tangential>::create(x,y);
  default:
    return rpe_no_distortion::create(x,y);
  }
}


} // end namespace ceres

} // end namespace maptk
} // end namespace kwiver


#endif // MAPTK_PLUGINS_CERES_REPROJECTION_ERROR_H_
