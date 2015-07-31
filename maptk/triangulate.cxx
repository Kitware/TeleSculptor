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
 * \brief Implementation of triangulation function
 */

#include "triangulate.h"
#include <boost/foreach.hpp>
#include <Eigen/SVD>


namespace kwiver {
namespace maptk {

/// Triangulate a 3D point from a set of cameras and 2D image points
template <typename T>
Eigen::Matrix<T,3,1>
triangulate_inhomog(const std::vector<kwiver::vital::camera_<T> >& cameras,
                    const std::vector<Eigen::Matrix<T,2,1> >& points)
{
  typedef Eigen::Matrix<T,2,1> vector_2;
  typedef Eigen::Matrix<T,3,1> vector_3;
  typedef Eigen::Matrix<T,3,3> matrix_3x3;
  typedef Eigen::Matrix<T, Eigen::Dynamic, 3> data_matrix_t;
  typedef Eigen::Matrix<T, Eigen::Dynamic, 1> data_vector_t;
  const unsigned int num_rows = 2*points.size();
  data_matrix_t A(num_rows, 3);
  data_vector_t b(num_rows);
  for( unsigned int i=0; i<points.size(); ++i )
  {
    // the camera
    const kwiver::vital::camera_<T>& cam = cameras[i];
    const matrix_3x3 R(cam.get_rotation());
    const vector_3 t(cam.get_translation());
    // the point in normalized coordinates
    const vector_2 pt = cam.get_intrinsics().unmap(points[i]);
    A(2*i,   0) = R(0,0) - pt.x() * R(2,0);
    A(2*i,   1) = R(0,1) - pt.x() * R(2,1);
    A(2*i,   2) = R(0,2) - pt.x() * R(2,2);
    A(2*i+1, 0) = R(1,0) - pt.y() * R(2,0);
    A(2*i+1, 1) = R(1,1) - pt.y() * R(2,1);
    A(2*i+1, 2) = R(1,2) - pt.y() * R(2,2);
    b[2*i  ] = t.z()*pt.x() - t.x();
    b[2*i+1] = t.z()*pt.y() - t.y();
  }
  Eigen::JacobiSVD<data_matrix_t> svd(A, Eigen::ComputeFullU |
                                         Eigen::ComputeFullV);
  return svd.solve(b);
}


/// Triangulate a homogeneous 3D point from a set of cameras and 2D image points
template <typename T>
Eigen::Matrix<T,4,1>
triangulate_homog(const std::vector<kwiver::vital::camera_<T> >& cameras,
                  const std::vector<Eigen::Matrix<T,2,1> >& points)
{
  typedef Eigen::Matrix<T,2,1> vector_2;
  typedef Eigen::Matrix<T,3,1> vector_3;
  typedef Eigen::Matrix<T,3,3> matrix_3x3;
  typedef Eigen::Matrix<T, Eigen::Dynamic, 4> data_matrix_t;
  const unsigned int num_rows = 2*points.size();
  data_matrix_t A(num_rows, 4);
  for( unsigned int i=0; i<points.size(); ++i )
  {
    // the camera
    const kwiver::vital::camera_<T>& cam = cameras[i];
    const matrix_3x3 R(cam.get_rotation());
    const vector_3 t(cam.get_translation());
    // the point in normalized coordinates
    const vector_2 pt = cam.get_intrinsics().unmap(points[i]);
    A(2*i,   0) = R(0,0) - pt.x() * R(2,0);
    A(2*i,   1) = R(0,1) - pt.x() * R(2,1);
    A(2*i,   2) = R(0,2) - pt.x() * R(2,2);
    A(2*i,   3) = t.x()  - pt.x() * t.z();
    A(2*i+1, 0) = R(1,0) - pt.y() * R(2,0);
    A(2*i+1, 1) = R(1,1) - pt.y() * R(2,1);
    A(2*i+1, 2) = R(1,2) - pt.y() * R(2,2);
    A(2*i+1, 3) = t.y()  - pt.y() * t.z();
  }
  Eigen::JacobiSVD<data_matrix_t > svd(A, Eigen::ComputeFullV);
  return svd.matrixV().col(3);
}



/// \cond DoxygenSuppress
#define INSTANTIATE_TRIANGULATE(T) \
template MAPTK_LIB_EXPORT Eigen::Matrix<T,4,1> \
         triangulate_homog(const std::vector<kwiver::vital::camera_<T> >& cameras, \
                           const std::vector<Eigen::Matrix<T,2,1> >& points); \
template MAPTK_LIB_EXPORT Eigen::Matrix<T,3,1> \
         triangulate_inhomog(const std::vector<kwiver::vital::camera_<T> >& cameras, \
                             const std::vector<Eigen::Matrix<T,2,1> >& points);

INSTANTIATE_TRIANGULATE(double);
INSTANTIATE_TRIANGULATE(float);

#undef INSTANTIATE_TRIANGULATE
/// \endcond


} // end namespace maptk
} // end namespace kwiver
