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
 * \brief Header for \link maptk::camera_intrinsics_ camera_intrinsics_<T>
 *        \endlink class
 */

#ifndef MAPTK_CAMERA_INTRINSICS_H_
#define MAPTK_CAMERA_INTRINSICS_H_

#include <maptk/config.h>

#include <iostream>

#include "matrix.h"
#include "vector.h"


namespace maptk
{

/// A representation of camera intrinsic parameters
template <typename T>
class MAPTK_LIB_EXPORT camera_intrinsics_
{
public:
  /// typedef for Eigen dynamic vector of type T
  typedef Eigen::Matrix<T, Eigen::Dynamic, 1> vector_t;

  /// Default Constructor
  camera_intrinsics_<T>()
  : focal_length_(T(1)),
    principal_point_(T(0), T(0)),
    aspect_ratio_(T(1)),
    skew_(T(0)),
    dist_coeffs_()
  {}

  /// Constructor for camera intrinsics
  camera_intrinsics_<T>(T focal_length,
                        const Eigen::Matrix<T,2,1>& principal_point,
                        T aspect_ratio=1.0,
                        T skew=0.0,
                        const vector_t dist_coeffs=vector_t())
  : focal_length_(focal_length),
    principal_point_(principal_point),
    aspect_ratio_(aspect_ratio),
    skew_(skew),
    dist_coeffs_(dist_coeffs)
  {}

  /// Copy Constructor from another type
  template <typename U>
  explicit camera_intrinsics_<T>(const camera_intrinsics_<U>& other)
  : focal_length_(static_cast<T>(other.focal_length())),
    principal_point_(other.principal_point().template cast<T>()),
    aspect_ratio_(static_cast<T>(other.aspect_ratio())),
    skew_(static_cast<T>(other.skew())),
    dist_coeffs_(other.dist_coeffs().template cast<T>())
  {}

  /// Constructor - from a calibration matrix
  /**
   * \note ignores values below the diagonal
   * \param K calibration matrix to construct from
   */
  explicit camera_intrinsics_<T>(const Eigen::Matrix<T,3,3>& K,
                                 const vector_t& d=vector_t());

  /// Access the focal length
  const T& focal_length() const { return focal_length_; }
  /// Access the principal point
  const Eigen::Matrix<T,2,1>& principal_point() const { return principal_point_; }
  /// Access the aspect ratio
  const T& aspect_ratio() const { return aspect_ratio_; }
  /// Access the skew
  const T& skew() const { return skew_; }
  /// Access the distortion coefficients
  const vector_t& dist_coeffs() const { return dist_coeffs_; }

  /// Set the focal length
  void set_focal_length(const T& focal_length) { focal_length_ = focal_length; }
  /// Set the principal point
  void set_principal_point(const Eigen::Matrix<T,2,1>& pp) { principal_point_ = pp; }
  /// Set the aspect_ratio
  void set_aspect_ratio(const T& aspect_ratio) { aspect_ratio_ = aspect_ratio; }
  /// Set the skew
  void set_skew(const T& skew) { skew_ = skew; }
  /// Set the distortion coefficients
  void set_dist_coeffs(const vector_t& d) { dist_coeffs_ = d; }

  /// Convert to a 3x3 calibration matrix
  operator Eigen::Matrix<T,3,3>() const;

  /// Map normalized image coordinates into actual image coordinates
  /** This function applies both distortion (if coefficients are specifed)
   *  and application of the calibration matrix to map into actual image
   *  coordinates
   */
  Eigen::Matrix<T,2,1> map(const Eigen::Matrix<T,2,1>& norm_pt) const;

  /// Map a 3D point in camera coordinates into actual image coordinates
  Eigen::Matrix<T,2,1> map(const Eigen::Matrix<T,3,1>& norm_hpt) const;

  /// Unmap actual image coordinates back into normalized image coordinates
  /** This function applies both application of the inverse calibration matrix
   *  and undistortion of the normalized coordinates (if distortion
   *  coefficients are specifed).
   */
  Eigen::Matrix<T,2,1> unmap(const Eigen::Matrix<T,2,1>& norm_pt) const;


  /// Map normalized image coordinates into distorted coordinates
  Eigen::Matrix<T,2,1> distort(const Eigen::Matrix<T,2,1>& norm_pt) const;

  /// Unnap distorted normalized coordinates into normalized coordinates
  /** \note applying inverse distortion is not closed form, so this function
   *  uses an iterative solver.
   */
  Eigen::Matrix<T,2,1> undistort(const Eigen::Matrix<T,2,1>& dist_pt) const;

protected:
  /// focal length of camera
  T focal_length_;
  /// principal point of camera
  Eigen::Matrix<T,2,1> principal_point_;
  /// aspect ratio of camera
  T aspect_ratio_;
  /// skew of camera
  T skew_;
  /// Lens distortion coefficients
  vector_t dist_coeffs_;
};


/// double-precision camera_intrinsics_ type
typedef camera_intrinsics_<double> camera_intrinsics_d;
/// single-precision camera_intrinsics_ type
typedef camera_intrinsics_<float> camera_intrinsics_f;

/// output stream operator for camera intrinsics
/**
 * \param s output stream
 * \param k camera_intrinsics_ to stream
 */
template <typename T>
MAPTK_LIB_EXPORT std::ostream& operator<<(std::ostream& s, const camera_intrinsics_<T>& k);

/// input stream operator for camera intrinsics
/**
 * \param s input stream
 * \param c camera_intrinsics_ to stream into
 */
template <typename T>
MAPTK_LIB_EXPORT std::istream& operator>>(std::istream& s, camera_intrinsics_<T>& c);


} // end namespace maptk


#endif // MAPTK_CAMERA_INTRINSICS_H_
