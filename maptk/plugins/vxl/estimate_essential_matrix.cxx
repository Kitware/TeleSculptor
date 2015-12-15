/*ckwg +29
 * Copyright 2014-2015 by Kitware, Inc.
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

#include "estimate_essential_matrix.h"

#include <vital/vital_foreach.h>

#include <vital/types/feature.h>
#include <maptk/plugins/vxl/camera.h>
#include <maptk/epipolar_geometry.h>

#include <vgl/vgl_point_2d.h>

#include <vpgl/algo/vpgl_em_compute_5_point.h>

using namespace kwiver::vital;

namespace kwiver {
namespace maptk {

namespace vxl
{


/// Private implementation class
class estimate_essential_matrix::priv
{
public:
  /// Constructor
  priv()
  : verbose(false),
    num_ransac_samples(512)
  {
  }

  priv(const priv& other)
  : verbose(other.verbose),
    num_ransac_samples(other.num_ransac_samples)
  {
  }

  bool verbose;
  unsigned num_ransac_samples;
};


/// Constructor
estimate_essential_matrix
::estimate_essential_matrix()
: d_(new priv)
{
}


/// Copy Constructor
estimate_essential_matrix
::estimate_essential_matrix(const estimate_essential_matrix& other)
: d_(new priv(*other.d_))
{
}


/// Destructor
estimate_essential_matrix
::~estimate_essential_matrix()
{
}



/// Get this algorithm's \link vital::config_block configuration block \endlink
vital::config_block_sptr
estimate_essential_matrix
::get_configuration() const
{
  // get base config from base class
  vital::config_block_sptr config =
      vital::algo::estimate_essential_matrix::get_configuration();

  config->set_value("verbose", d_->verbose,
                    "If true, write status messages to the terminal showing "
                    "debugging information");

  config->set_value("num_ransac_samples", d_->num_ransac_samples,
                    "The number of samples to use in RANSAC");

  return config;
}


/// Set this algorithm's properties via a config block
void
estimate_essential_matrix
::set_configuration(vital::config_block_sptr config)
{

  d_->verbose = config->get_value<bool>("verbose",
                                        d_->verbose);
  d_->num_ransac_samples = config->get_value<unsigned>("num_ransac_samples",
                                                       d_->num_ransac_samples);
}


/// Check that the algorithm's currently configuration is valid
bool
estimate_essential_matrix
::check_configuration(vital::config_block_sptr config) const
{
  return true;
}


/// Estimate an essential matrix from corresponding points
essential_matrix_sptr
estimate_essential_matrix
::estimate(const std::vector<vector_2d>& pts1,
           const std::vector<vector_2d>& pts2,
           const camera_intrinsics_sptr cal1,
           const camera_intrinsics_sptr cal2,
           std::vector<bool>& inliers,
           double inlier_scale) const
{
  vpgl_calibration_matrix<double> vcal1, vcal2;
  maptk_to_vpgl_calibration(*cal1, vcal1);
  maptk_to_vpgl_calibration(*cal2, vcal2);

  vcl_vector<vgl_point_2d<double> > right_points, left_points;
  VITAL_FOREACH(const vector_2d& v, pts1)
  {
    right_points.push_back(vgl_point_2d<double>(v.x(), v.y()));
  }
  VITAL_FOREACH(const vector_2d& v, pts2)
  {
    left_points.push_back(vgl_point_2d<double>(v.x(), v.y()));
  }

  double sq_scale = inlier_scale * inlier_scale;
  vpgl_em_compute_5_point_ransac<double> em(d_->num_ransac_samples, sq_scale,
                                            d_->verbose);
  vpgl_essential_matrix<double> best_em;
  em.compute(right_points, vcal1, left_points, vcal2, best_em);

  matrix_3x3d E(best_em.get_matrix().data_block());
  E.transposeInPlace();
  matrix_3x3d K1_inv = cal1->as_matrix().inverse();
  matrix_3x3d K2_invt = cal2->as_matrix().transpose().inverse();
  matrix_3x3d F = K2_invt * E * K1_inv;

  fundamental_matrix_sptr fm(new fundamental_matrix_d(F));
  inliers = maptk::mark_fm_inliers(fm, pts1, pts2, inlier_scale);

  return essential_matrix_sptr(new essential_matrix_d(E));
}


} // end namespace vxl

} // end namespace maptk
} // end namespace kwiver
