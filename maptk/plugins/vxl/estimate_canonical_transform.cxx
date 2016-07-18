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

#include <vnl/vnl_double_3.h>
#include <rrel/rrel_irls.h>
#include <rrel/rrel_lms_obj.h>
#include <rrel/rrel_orthogonal_regression.h>
#include <rrel/rrel_ran_sam_search.h>
#include <rrel/rrel_ransac_obj.h>
#include <rrel/rrel_tukey_obj.h>

using namespace kwiver::vital;

namespace kwiver {
namespace maptk {
namespace vxl {


/// Private implementation class
class estimate_canonical_transform::priv
{
public:
  enum rrel_method_types {RANSAC, LMS, IRLS};
  /// Constructor
  priv()
    : estimate_scale(true),
      trace_level(0),
      rrel_method(IRLS),
      desired_prob_good(0.99),
      max_outlier_frac(0.75),
      prior_inlier_scale(0.1),
      irls_max_iterations(15),
      irls_iterations_for_scale(2),
      irls_conv_tolerance(1e-4),
      m_logger( vital::get_logger( "maptk.vxl.estimate_canonical_transform" ))
  {
  }

  priv(const priv& other)
    : estimate_scale(other.estimate_scale),
      trace_level(other.trace_level),
      rrel_method(other.rrel_method),
      desired_prob_good(other.desired_prob_good),
      max_outlier_frac(other.max_outlier_frac),
      prior_inlier_scale(other.prior_inlier_scale),
      irls_max_iterations(other.irls_max_iterations),
      irls_iterations_for_scale(other.irls_iterations_for_scale),
      irls_conv_tolerance(other.irls_conv_tolerance),
      m_logger( vital::get_logger( "maptk.vxl.estimate_canonical_transform" ))
  {
  }

  /// Helper function to estimate a ground plane from the data points
  vector_4d estimate_plane(std::vector<vector_3d> const& points) const
  {
    std::vector< vnl_vector<double> > vnl_points;
    VITAL_FOREACH(vector_3d const& p, points)
    {
      vnl_points.push_back(vnl_vector<double>(vnl_double_3(p[0], p[1], p[2])));
    }

    // The number of different populations in the data set. For most
    // problems, the data is from one source (surface, etc.), so this
    // will be 1.
    int max_pops = 1;

    rrel_orthogonal_regression *reg = new rrel_orthogonal_regression(vnl_points);
    vnl_vector<double> pp;

    switch(rrel_method)
    {
      case RANSAC:
      {
        rrel_ransac_obj* ransac = new rrel_ransac_obj();
        rrel_ran_sam_search* ransam = new rrel_ran_sam_search;
        ransam->set_sampling_params(max_outlier_frac, desired_prob_good, max_pops);
        ransam->set_trace_level(trace_level);

        reg->set_prior_scale(prior_inlier_scale);

        if ( !ransam->estimate( reg, ransac) )
        {
          LOG_ERROR(m_logger, "RANSAC unable to fit a plane to the landmarks.");
        }
        LOG_DEBUG(m_logger, "Estimated scale = " << ransam->scale());
        pp = ransam->params();
        delete ransam;
        delete ransac;

      }
      case LMS:
      {
        int num_sam_inst = reg->num_samples_to_instantiate();
        rrel_objective* lms = new rrel_lms_obj( num_sam_inst );
        rrel_ran_sam_search* ransam = new rrel_ran_sam_search;
        ransam->set_sampling_params(max_outlier_frac, desired_prob_good, max_pops);
        ransam->set_trace_level(trace_level);

        if ( !ransam->estimate( reg, lms) )
        {
          LOG_ERROR(m_logger, "LMS unable to fit a plane to the landmarks.");
        }
        LOG_DEBUG(m_logger, "Estimated scale = " << ransam->scale());
        pp = ransam->params();
        delete ransam;
        delete lms;
      }
      case IRLS:
      {
        //  Beaton-Tukey loss function
        rrel_m_est_obj * m_est = new rrel_tukey_obj( 4.0 );

        reg->set_no_prior_scale();

        // Iteratively Reweighted Least Squares
        rrel_irls* irls = new rrel_irls( irls_max_iterations );
        irls->set_est_scale( irls_iterations_for_scale );
        irls->set_convergence_test( irls_conv_tolerance );
        irls->set_trace_level(trace_level);

        if ( !irls->estimate( reg, m_est ) )
        {
          LOG_ERROR(m_logger, "IRLS unable to fit a plane to the landmarks.");
        }
        LOG_DEBUG(m_logger, "Estimated scale = " << irls->scale());
        pp = irls->params();
        delete irls;
        delete m_est;
      }
    }

    delete reg;
    return vector_4d(pp[0], pp[1], pp[2], pp[3]);
  }

  // Enable estimation of scale in the similarity transform
  bool estimate_scale;
  // This controls the verbosity of the search techniques.
  int trace_level;
  // The robust estimation method to used
  rrel_method_types rrel_method;
  // The desired probability of finding the correct fit.
  double desired_prob_good;
  // The maximum fraction of the data that is expected to be gross outliers.
  double max_outlier_frac;
  // The initial estimate of inlier scale for RANSAC
  double prior_inlier_scale;
  // The maximum number of iterations for IRLS
  int irls_max_iterations;
  // The number of IRLS iterations in which to estimate scale
  int irls_iterations_for_scale;
  // The convergence tolerance for IRLS
  double irls_conv_tolerance;
  // Logger handle
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

  config->set_value("trace_level", d_->trace_level,
                    "Integer value controlling the verbosity of the "
                    "plane search algorithms (0->no output, 3->max output).");

  config->set_value("rrel_method", static_cast<int>(d_->rrel_method),
                    "The robust estimation algorithm to use for plane "
                    "fitting. Options are:\n"
                    " 0 = RANSAC\n"
                    " 1 = Least Median of Squares (LMS)\n"
                    " 2 = Iteratively Reweighted Least Squares (IRLS)");

  config->set_value("desired_prob_good", d_->desired_prob_good,
                    "The desired probability of finding the correct plane fit.");

  config->set_value("max_outlier_frac", d_->max_outlier_frac,
                    "The maximum fraction of the landmarks that is expected "
                    "outliers to the ground plane.");

  config->set_value("prior_inlier_scale", d_->prior_inlier_scale,
                    "The initial estimate of inlier scale for RANSAC "
                    "fitting of the ground plane.");

  config->set_value("irls_max_iterations", d_->irls_max_iterations,
                    "The maximum number if iterations when using IRLS");

  config->set_value("irls_iterations_for_scale", d_->irls_iterations_for_scale,
                    "The number of IRLS iterations in which to estimate scale");

  config->set_value("irls_conv_tolerance", d_->irls_conv_tolerance,
                    "The convergence tolerance for IRLS");
  return config;
}


/// Set this algorithm's properties via a config block
void
estimate_canonical_transform
::set_configuration(vital::config_block_sptr config)
{
  d_->estimate_scale = config->get_value<bool>("estimate_scale", d_->estimate_scale);
  d_->trace_level = config->get_value<int>("trace_level", d_->trace_level);
  d_->rrel_method = static_cast<priv::rrel_method_types>(
                        config->get_value<int>("rrel_method", static_cast<int>(d_->rrel_method)));
  d_->desired_prob_good = config->get_value<double>("desired_prob_good", d_->desired_prob_good);
  d_->max_outlier_frac = config->get_value<double>("max_outlier_frac", d_->max_outlier_frac);
  d_->prior_inlier_scale = config->get_value<double>("prior_inlier_scale", d_->prior_inlier_scale);
  d_->irls_max_iterations = config->get_value<int>("irls_max_iterations", d_->irls_max_iterations);
  d_->irls_iterations_for_scale = config->get_value<int>("irls_iterations_for_scale", d_->irls_iterations_for_scale);
  d_->irls_conv_tolerance = config->get_value<double>("irls_conv_tolerance", d_->irls_conv_tolerance);
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
