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
 * \brief Implementation of epipolar geometry functions.
 */

#include "epipolar_geometry.h"
#include <vital/vital_foreach.h>
#include <maptk/triangulate.h>


namespace kwiver {
namespace maptk {


/// Test corresponding points against a fundamental matrix and mark inliers
std::vector<bool>
mark_fm_inliers(vital::fundamental_matrix const& fm,
                std::vector<vital::vector_2d> const& pts1,
                std::vector<vital::vector_2d> const& pts2,
                double inlier_scale)
{
  using namespace kwiver::vital;

  matrix_3x3d F = fm.matrix();
  matrix_3x3d Ft = F.transpose();

  std::vector<bool> inliers(std::min(pts1.size(), pts2.size()));
  for(unsigned i=0; i<inliers.size(); ++i)
  {
    const vector_2d& p1 = pts1[i];
    const vector_2d& p2 = pts2[i];
    vector_3d v1(p1.x(), p1.y(), 1.0);
    vector_3d v2(p2.x(), p2.y(), 1.0);
    vector_3d l1 = F * v1;
    vector_3d l2 = Ft * v2;
    double s1 = 1.0 / sqrt(l1.x()*l1.x() + l1.y()*l1.y());
    double s2 = 1.0 / sqrt(l2.x()*l2.x() + l2.y()*l2.y());
    // sum of point to epipolar line distance in both images
    double d = v1.dot(l2) * (s1 + s2);
    inliers[i] = std::fabs(d) < inlier_scale;
  }
  return inliers;
}


/// Compute a valid left camera from an essential matrix
kwiver::vital::simple_camera
extract_valid_left_camera(const kwiver::vital::essential_matrix_d& e,
                          const kwiver::vital::vector_2d& left_pt,
                          const kwiver::vital::vector_2d& right_pt)
{
  using namespace kwiver::vital;

  /// construct an identity right camera
  const vector_3d t = e.translation();
  rotation_d R = e.rotation();

  std::vector<vector_2d> pts;
  pts.push_back(right_pt);
  pts.push_back(left_pt);

  std::vector<vital::simple_camera> cams(2);
  const vital::simple_camera& left_camera = cams[1];

  // option 1
  cams[1] = vital::simple_camera(R.inverse()*-t, R);
  vector_3d pt3 = triangulate_inhomog(cams, pts);
  if( pt3.z() > 0.0 && left_camera.depth(pt3) > 0.0 )
  {
    return left_camera;
  }

  // option 2, with negated translation
  cams[1] = vital::simple_camera(R.inverse()*t, R);
  pt3 = triangulate_inhomog(cams, pts);
  if( pt3.z() > 0.0 && left_camera.depth(pt3) > 0.0 )
  {
    return left_camera;
  }

  // option 3, with the twisted pair rotation
  R = e.twisted_rotation();
  cams[1] = vital::simple_camera(R.inverse()*-t, R);
  pt3 = triangulate_inhomog(cams, pts);
  if( pt3.z() > 0.0 && left_camera.depth(pt3) > 0.0 )
  {
    return left_camera;
  }

  // option 4, with negated translation
  cams[1] = vital::simple_camera(R.inverse()*t, R);
  pt3 = triangulate_inhomog(cams, pts);
  if( pt3.z() > 0.0 && left_camera.depth(pt3) > 0.0 )
  {
    return left_camera;
  }
  // should never get here
  return vital::simple_camera();
}



// Compute the fundamental matrix from a pair of cameras
kwiver::vital::fundamental_matrix_sptr
fundamental_matrix_from_cameras(kwiver::vital::camera const& right_cam,
                                kwiver::vital::camera const& left_cam)
{
  using namespace kwiver::vital;
  essential_matrix_sptr em = essential_matrix_from_cameras(right_cam, left_cam);
  return essential_matrix_to_fundamental(*em, *right_cam.intrinsics(),
                                              *left_cam.intrinsics());
}


// Compute the essential matrix from a pair of cameras
kwiver::vital::essential_matrix_sptr
essential_matrix_from_cameras(kwiver::vital::camera const& right_cam,
                              kwiver::vital::camera const& left_cam)
{
  using namespace kwiver::vital;
  rotation_d R1 = right_cam.rotation();
  rotation_d R2 = left_cam.rotation();
  vector_3d t1 = right_cam.translation();
  vector_3d t2 = left_cam.translation();
  rotation_d R(R2 * R1.inverse());
  vector_3d t(t2 - R*t1);
  return std::make_shared<essential_matrix_d>(R,t);
}


/// Convert an essential matrix to a fundamental matrix
kwiver::vital::fundamental_matrix_sptr
essential_matrix_to_fundamental(kwiver::vital::essential_matrix const & E,
                                kwiver::vital::camera_intrinsics const& right_cal,
                                kwiver::vital::camera_intrinsics const& left_cal)
{
  using namespace kwiver::vital;
  matrix_3x3d Kr_inv = right_cal.as_matrix().inverse();
  matrix_3x3d Kl_invt = left_cal.as_matrix().transpose().inverse();
  return std::make_shared<fundamental_matrix_d>( Kl_invt * E.matrix() * Kr_inv );
}



} // end namespace maptk
} // end namespace kwiver
