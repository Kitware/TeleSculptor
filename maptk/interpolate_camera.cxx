/*ckwg +29
 * Copyright 2013-2016 by Kitware, Inc.
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
 * \brief Implementation of camera interpolation functions
 */

#include "interpolate_camera.h"


namespace kwiver {
namespace maptk {


/// Generate an interpolated camera between \c A and \c B by a given fraction \c f
vital::simple_camera
interpolate_camera(vital::simple_camera const& A,
                   vital::simple_camera const& B, double f)
{
  const double f1 = 1.0 - f;

  // interpolate center
  vital::vector_3d c = f1*A.get_center() + f*B.get_center();

  // interpolate rotation
  vital::rotation_d R = interpolate_rotation(A.get_rotation(), B.get_rotation(), f);

  // interpolate intrinsics
  vital::camera_intrinsics_sptr k1 = A.get_intrinsics(),
                                k2 = B.get_intrinsics();

  if( k1 == k2 )
  {
    return vital::simple_camera(c, R, k1);
  }

  double focal_len = f1*k1->focal_length() + f*k2->focal_length();
  vital::vector_2d principal_point = f1*k1->principal_point() + f*k2->principal_point();
  double aspect_ratio = f1*k1->aspect_ratio() + f*k2->aspect_ratio();
  double skew = f1*k1->skew() + f*k2->skew();
  vital::simple_camera_intrinsics k(focal_len, principal_point, aspect_ratio, skew);
  return vital::simple_camera(c, R, k);
}


/// Generate N evenly interpolated cameras in between \c A and \c B
void
interpolated_cameras(vital::simple_camera const& A,
                     vital::simple_camera const& B,
                     size_t n,
                     std::vector< vital::simple_camera > & interp_cams)
{
  interp_cams.reserve(interp_cams.capacity() + n);
  size_t denom = n + 1;
  for (size_t i=1; i < denom; ++i)
  {
    interp_cams.push_back(interpolate_camera(A, B, static_cast<double>(i) / denom));
  }
}


/// Genreate an interpolated camera from sptrs
vital::camera_sptr
interpolate_camera(vital::camera_sptr A,
                   vital::camera_sptr B, double f)
{
  if( A == B )
  {
    return A;
  }
  return interpolate_camera(vital::simple_camera(*A),
                            vital::simple_camera(*B), f).clone();
}


} // end namespace maptk
} // end namespace kwiver
