/*ckwg +29
 * Copyright 2014 by Kitware, Inc.
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
 * \brief vxl estimate essential matrix implementation
 */

#include <maptk/vxl/estimate_essential_matrix.h>
#include <maptk/core/feature.h>
#include <vpgl/algo/vpgl_em_compute_5_point.h>
#include <vgl/vgl_point_2d.h>


namespace maptk
{

namespace vxl
{

/// Estimate an essential matrix from corresponding points
matrix_3x3d
estimate_essential_matrix
::estimate(feature_set_sptr feat1,
           feature_set_sptr feat2,
           match_set_sptr matches,
           const camera_intrinsics_d &ci) const
{
  vpgl_calibration_matrix<double> cal;
  cal.set_focal_length(ci.focal_length());
  cal.set_principal_point(vgl_point_2d<double>(ci.principal_point()[0], ci.principal_point()[1]));
  cal.set_x_scale(ci.aspect_ratio());
  cal.set_y_scale(1.0);
  cal.set_skew(ci.skew());

  vcl_vector<vgl_point_2d<double> > right_points, left_points;
  for (unsigned int i = 0; i < matches->size(); i++)
  {
    const feature_sptr f1 = feat1->features()[matches->matches()[i].first];
    const feature_sptr f2 = feat2->features()[matches->matches()[i].second];
    right_points.push_back(vgl_point_2d<double>(f1->loc().x(), f1->loc().y()));
    left_points.push_back(vgl_point_2d<double>(f2->loc().x(), f2->loc().y()));
  }

  vpgl_em_compute_5_point_ransac<double> em;
  vpgl_essential_matrix<double> best_em;
  em.compute(right_points, cal, left_points, cal, best_em);

  return matrix_3x3d(best_em.get_matrix().data_block());
}


} // end namespace vxl

} // end namespace maptk
