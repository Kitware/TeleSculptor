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
 * \brief Implementation of evaluation metric functions.
 */

#include "epipolar_geometry.h"
#include <vital/vital_foreach.h>


namespace kwiver {
namespace maptk {


/// Test corresponding points against a fundamental matrix and mark inliers
std::vector<bool>
mark_fm_inliers(vital::fundamental_matrix_sptr const& fm,
                std::vector<vital::vector_2d> const& pts1,
                std::vector<vital::vector_2d> const& pts2,
                double inlier_scale)
{
  using namespace kwiver::vital;

  matrix_3x3d F = fm->matrix();
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


} // end namespace maptk
} // end namespace kwiver
