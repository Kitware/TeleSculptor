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
 * \brief Header for lens distorion functions
 */

#ifndef MAPTK_PLUGINS_CERES_LENS_DISTORTION_H_
#define MAPTK_PLUGINS_CERES_LENS_DISTORTION_H_


#include <maptk/plugins/ceres/ceres_config.h>


namespace maptk
{

namespace ceres
{


/// Class to hold to distortion function and traits
class distortion_poly_radial
{
public:
  // the number of distortion coefficients
  static const int num_coeffs = 2;

  /// Function to apply polynomial radial distortion
  /**
   * \param [in] dist_coeffs: radial distortion coefficients (2)
   * \param [in] source_xy: 2D point in normalized image coordinates
   * \param [out] distorted_xy: 2D point in distorted normalized image coordinates
   */
  template <typename T>
  static void apply(const T* dist_coeffs,
                    const T* source_xy,
                          T* distorted_xy)
  {
    const T& x = source_xy[0];
    const T& y = source_xy[1];

    // distortion parameters
    const T& k1 = dist_coeffs[0];
    const T& k2 = dist_coeffs[1];

    // apply radial distortion
    const T r2 = x*x + y*y;
    const T scale = T(1) + k1*r2 + k2*r2*r2;
    distorted_xy[0] = x*scale;
    distorted_xy[1] = y*scale;
  }
};


/// Class to hold to distortion function and traits
class distortion_poly_radial_tangential
{
public:
  // the number of distortion coefficients
  static const int num_coeffs = 5;

  /// Function to apply polynomial radial and tangential distortion
  /**
   * \param [in] dist_coeffs: radial (3) and tangential (2) distortion
   *                            coefficients
   * \param [in] source_xy: 2D point in normalized image coordinates
   * \param [out] distorted_xy: 2D point in distorted normalized image coordinates
   */
  template <typename T>
  static void apply(const T* dist_coeffs,
                    const T* source_xy,
                          T* distorted_xy)
  {
    const T& x = source_xy[0];
    const T& y = source_xy[1];

    // distortion parameters
    const T& k1 = dist_coeffs[0];
    const T& k2 = dist_coeffs[1];
    const T& p1 = dist_coeffs[2];
    const T& p2 = dist_coeffs[3];
    const T& k3 = dist_coeffs[4];

    // apply radial distortion
    const T x2 = x*x;
    const T y2 = y*y;
    const T xy = x*y;
    const T r2 = x2 + y2;
    const T r4 = r2*r2;
    const T scale = T(1) + k1*r2 + k2*r4 + k3*r2*r4;
    distorted_xy[0] = x*scale + T(2)*p1*xy + p2*(r2 + T(2)*x2);
    distorted_xy[1] = y*scale + T(2)*p2*xy + p1*(r2 + T(2)*y2);
  }
};


/// Class to hold to distortion function and traits
class distortion_ratpoly_radial_tangential
{
public:
  // the number of distortion coefficients
  static const int num_coeffs = 8;

  /// Function to apply rational polynomial radial and tangential distortion
  /**
   * \param [in] dist_coeffs: radial (6) and tangential (2) distortion
   *                          coefficients
   * \param [in] source_xy: 2D point in normalized image coordinates
   * \param [out] distorted_xy: 2D point in distorted normalized image coordinates
   */
  template <typename T>
  static void apply(const T* dist_coeffs,
                    const T* source_xy,
                          T* distorted_xy)
  {
    const T& x = source_xy[0];
    const T& y = source_xy[1];

    // distortion parameters
    const T& k1 = dist_coeffs[0];
    const T& k2 = dist_coeffs[1];
    const T& p1 = dist_coeffs[2];
    const T& p2 = dist_coeffs[3];
    const T& k3 = dist_coeffs[4];
    const T& k4 = dist_coeffs[5];
    const T& k5 = dist_coeffs[6];
    const T& k6 = dist_coeffs[7];

    // apply radial distortion
    const T x2 = x*x;
    const T y2 = y*y;
    const T xy = x*y;
    const T r2 = x2 + y2;
    const T r4 = r2*r2;
    const T r6 = r4*r2;
    const T scale = (T(1) + k1*r2 + k2*r4 + k3*r6) /
                    (T(1) + k4*r2 + k5*r4 + k6*r6);
    distorted_xy[0] = x*scale + T(2)*p1*xy + p2*(r2 + T(2)*x2);
    distorted_xy[1] = y*scale + T(2)*p2*xy + p1*(r2 + T(2)*y2);
  }
};


} // end namespace ceres

} // end namespace maptk


#endif // MAPTK_PLUGINS_CERES_LENS_DISTORTION_H_
