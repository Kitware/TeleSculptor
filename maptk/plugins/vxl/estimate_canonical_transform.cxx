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
 * \brief Implementation of VXL estimate_canonical_transform algorithm
 */

#include "estimate_canonical_transform.h"

#include <vital/logger/logger.h>

#include <algorithm>

#include <vgl/algo/vgl_fit_plane_3d.h>

using namespace kwiver::vital;

namespace kwiver {
namespace maptk {
namespace vxl {


/// Private implementation class
class estimate_canonical_transform::priv
{
public:
  /// Constructor
  priv()
    : estimate_scale(true),
      m_logger( vital::get_logger( "estimate_canonical_transform" ))
  {
  }

  priv(const priv& other)
    : estimate_scale(other.estimate_scale),
      m_logger( vital::get_logger( "estimate_canonical_transform" ))
  {
  }

  /// Helper function to estimate a ground plane from the data points
  vector_4d estimate_plane(std::vector<vector_3d> const& points) const
  {
    vgl_fit_plane_3d<double> fit;
    VITAL_FOREACH(vector_3d const& p, points)
    {
      fit.add_point(p[0], p[1], p[2]);
    }
    if( !fit.fit() )
    {
      LOG_ERROR(m_logger, "Unable to fit a plane to the landmarks.");
    }
    vgl_homg_plane_3d<double> plane = fit.get_plane();

    return vector_4d(plane.a(), plane.b(), plane.c(), plane.d());
  }

  bool estimate_scale;
  vital::logger_handle_t m_logger;
};


/// Constructor
estimate_canonical_transform
::estimate_canonical_transform()
: d_(new priv)
{
}


/// Copy Constructor
estimate_canonical_transform
::estimate_canonical_transform(const estimate_canonical_transform& other)
: d_(new priv(*other.d_))
{
}


/// Destructor
estimate_canonical_transform
::~estimate_canonical_transform()
{
}


/// Get this algorithm's \link vital::config_block configuration block \endlink
  vital::config_block_sptr
estimate_canonical_transform
::get_configuration() const
{
  // get base config from base class
  vital::config_block_sptr config =
      vital::algo::estimate_canonical_transform::get_configuration();

  config->set_value("estimate_scale", d_->estimate_scale,
                    "Estimate the scale to normalize the data. "
                    "If disabled the estimate transform is rigid");

  return config;
}


/// Set this algorithm's properties via a config block
void
estimate_canonical_transform
::set_configuration(vital::config_block_sptr config)
{
  d_->estimate_scale = config->get_value<bool>("estimate_scale", d_->estimate_scale);
}


/// Check that the algorithm's configuration vital::config_block is valid
bool
estimate_canonical_transform
::check_configuration(vital::config_block_sptr config) const
{
 return true;
}


/// Estimate a canonical similarity transform for cameras and points
kwiver::vital::similarity_d
estimate_canonical_transform
::estimate_transform(kwiver::vital::camera_map_sptr const cameras,
                     kwiver::vital::landmark_map_sptr const landmarks) const
{
  using namespace maptk;
  std::vector<vector_3d> points;
  VITAL_FOREACH(auto const& p, landmarks->landmarks())
  {
    points.push_back(p.second->loc());
  }
  // estimate the ground plane
  vector_4d plane = d_->estimate_plane(points);
  vector_3d normal = plane.head<3>();

  //project the points onto the plane
  VITAL_FOREACH(vector_3d& p, points)
  {
    p -= (normal.dot(p) + plane[3]) * normal;
  }

  // find the centroid and scale of all the landmarks
  vital::vector_3d center(0,0,0);
  double s=0.0;
  vital::matrix_3x3d covar = vital::matrix_3x3d::Zero();
  VITAL_FOREACH(vector_3d const& p, points)
  {
    center += p;
    covar += p * p.transpose();
    s += p.dot(p);
  }
  const double num_lm = static_cast<double>(points.size());
  center /= num_lm;
  covar /= num_lm;
  covar -= center * center.transpose();
  s /= num_lm;
  s -= center.dot(center);
  s = 1.0/std::sqrt(s);

  Eigen::JacobiSVD<vital::matrix_3x3d> svd(covar, Eigen::ComputeFullV);
  vital::matrix_3x3d rot = svd.matrixV();
  // ensure that rot is a rotation (determinant 1)
  rot.col(1) = rot.col(2).cross(rot.col(0)).normalized();

  if(cameras->size() > 0)
  {
    // find the average camera center and  average up direction
    vital::vector_3d cam_center(0,0,0);
    vital::vector_3d cam_up(0,0,0);
    typedef vital::camera_map::map_camera_t cam_map_t;
    VITAL_FOREACH(const cam_map_t::value_type& p, cameras->cameras())
    {
      cam_center += p.second->center();
    }
    cam_center /= static_cast<double>(cameras->size());
    cam_center -= center;
    cam_center = cam_center.normalized();
    // flip the plane normal if it points away from the cameras
    if( cam_center.dot(rot.col(2)) < 0.0 )
    {
      // rotate 180 about the X-axis
      rot.col(2) = -rot.col(2);
      rot.col(1) = -rot.col(1);
    }
  }

  if(!d_->estimate_scale)
  {
    s = 1.0;
  }

  vital::rotation_d R(rot);
  R = R.inverse();
  return vital::similarity_d(s, R, R*(-s*center));
}


} // end namespace vxl
} // end namespace maptk
} // end namespace kwiver
