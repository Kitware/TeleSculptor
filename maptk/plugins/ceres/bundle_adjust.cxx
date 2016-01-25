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
 * \brief Implementation of Ceres bundle adjustment algorithm
 */

#include "bundle_adjust.h"

#include <iostream>
#include <set>

#include <vital/vital_foreach.h>
#include <boost/timer/timer.hpp>

#include <vital/logger/logger.h>
#include <vital/io/eigen_io.h>
#include <maptk/plugins/ceres/reprojection_error.h>
#include <maptk/plugins/ceres/types.h>

#include <ceres/ceres.h>



using boost::timer::cpu_times;
using boost::timer::nanosecond_type;
using namespace kwiver::vital;


#define MAPTK_CERES_ENUM_HELPERS(NS, ceres_type)                        \
namespace kwiver {                                                      \
namespace vital {                                                       \
                                                                        \
template<>                                                              \
inline                                                                  \
config_block_value_t                                                    \
config_block_set_value_cast(NS::ceres_type const& value)                \
{                                                                       \
  return NS::ceres_type##ToString(value);                               \
}                                                                       \
                                                                        \
template<>                                                              \
inline                                                                  \
NS::ceres_type                                                          \
config_block_get_value_cast(config_block_value_t const& value)          \
{                                                                       \
  NS::ceres_type cet;                                                   \
  if(!NS::StringTo##ceres_type(value, &cet))                            \
  {                                                                     \
    throw bad_config_block_cast(value);                                 \
  }                                                                     \
  return cet;                                                           \
}                                                                       \
                                                                        \
}                                                                       \
                                                                        \
namespace maptk {                                                       \
namespace ceres {                                                       \
                                                                        \
template<>                                                              \
std::string                                                             \
ceres_options< NS::ceres_type >()                                       \
{                                                                       \
  typedef NS::ceres_type T;                                             \
  std::string options_str = "\nMust be one of the following options:";  \
  std::string opt;                                                      \
  for (unsigned i=0; i<20; ++i)                                         \
  {                                                                     \
    opt = NS::ceres_type##ToString(static_cast<T>(i));                  \
    if (opt == "UNKNOWN")                                               \
    {                                                                   \
      break;                                                            \
    }                                                                   \
    options_str += "\n  - " + opt;                                      \
  }                                                                     \
  return options_str;                                                   \
}                                                                       \
                                                                        \
}                                                                       \
}                                                                       \
}

namespace kwiver {
namespace maptk {
namespace ceres {

/// Defult implementation of string options for Ceres enums
template <typename T>
std::string
ceres_options()
{
  return std::string();
}

} // end namespace ceres
} // end namespace maptk
} // end namespace kwiver

MAPTK_CERES_ENUM_HELPERS(::ceres, LinearSolverType)
MAPTK_CERES_ENUM_HELPERS(::ceres, PreconditionerType)
MAPTK_CERES_ENUM_HELPERS(::ceres, TrustRegionStrategyType)
MAPTK_CERES_ENUM_HELPERS(::ceres, DoglegType)

MAPTK_CERES_ENUM_HELPERS(kwiver::maptk::ceres, LossFunctionType)
MAPTK_CERES_ENUM_HELPERS(kwiver::maptk::ceres, LensDistortionType)
MAPTK_CERES_ENUM_HELPERS(kwiver::maptk::ceres, CameraIntrinsicShareType)

#undef MAPTK_CERES_ENUM_HELPERS


namespace kwiver {
namespace maptk {
namespace ceres {

/// Private implementation class
class bundle_adjust::priv
{
public:
  /// Constructor
  priv()
  : verbose(false),
    loss_function_type(TRIVIAL_LOSS),
    loss_function_scale(1.0),
    optimize_focal_length(true),
    optimize_aspect_ratio(false),
    optimize_principal_point(false),
    optimize_skew(false),
    lens_distortion_type(NO_DISTORTION),
    optimize_dist_k1(true),
    optimize_dist_k2(false),
    optimize_dist_k3(false),
    optimize_dist_p1_p2(false),
    optimize_dist_k4_k5_k6(false),
    camera_intrinsic_share_type(AUTO_SHARE_INTRINSICS),
    m_logger( vital::get_logger( "maptk::ceres::bundle_adjust" ))
  {
  }

  priv(const priv& other)
  : verbose(other.verbose),
    loss_function_type(other.loss_function_type),
    loss_function_scale(other.loss_function_scale),
    optimize_focal_length(other.optimize_focal_length),
    optimize_aspect_ratio(other.optimize_aspect_ratio),
    optimize_principal_point(other.optimize_principal_point),
    optimize_skew(other.optimize_skew),
    lens_distortion_type(other.lens_distortion_type),
    optimize_dist_k1(other.optimize_dist_k1),
    optimize_dist_k2(other.optimize_dist_k2),
    optimize_dist_k3(other.optimize_dist_k3),
    optimize_dist_p1_p2(other.optimize_dist_p1_p2),
    optimize_dist_k4_k5_k6(other.optimize_dist_k4_k5_k6),
    camera_intrinsic_share_type(other.camera_intrinsic_share_type),
    m_logger( vital::get_logger( "maptk::ceres::bundle_adjust" ))
  {
  }

  /// the Ceres solver options
  ::ceres::Solver::Options options;
  /// verbose output
  bool verbose;
  /// the robust loss function type to use
  LossFunctionType loss_function_type;
  /// the scale of the loss function
  double loss_function_scale;
  /// option to optimize the focal length
  bool optimize_focal_length;
  /// option to optimize aspect ratio
  bool optimize_aspect_ratio;
  /// option to optimize principal point
  bool optimize_principal_point;
  /// option to optimize skew
  bool optimize_skew;
  /// the lens distortion model to use
  LensDistortionType lens_distortion_type;
  /// option to optimize radial distortion parameter k1
  bool optimize_dist_k1;
  /// option to optimize radial distortion parameter k2
  bool optimize_dist_k2;
  /// option to optimize radial distortion parameter k3
  bool optimize_dist_k3;
  /// option to optimize tangential distortions parameters p1, p2
  bool optimize_dist_p1_p2;
  /// option to optimize radial distortion parameters k4, k5, k6
  bool optimize_dist_k4_k5_k6;
  /// the type of sharing of intrinsics between cameras to use
  CameraIntrinsicShareType camera_intrinsic_share_type;

  /// Logger handle
  vital::logger_handle_t m_logger;
};


/// Constructor
bundle_adjust
::bundle_adjust()
: d_(new priv)
{
}


/// Copy Constructor
bundle_adjust
::bundle_adjust(const bundle_adjust& other)
: d_(new priv(*other.d_))
{
}


/// Destructor
bundle_adjust
::~bundle_adjust()
{
}


/// Get this algorithm's \link maptk::config_block configuration block \endlink
config_block_sptr
bundle_adjust
::get_configuration() const
{
  ::ceres::Solver::Options& o = d_->options;
  // get base config from base class
  config_block_sptr config = vital::algo::bundle_adjust::get_configuration();
  config->set_value("verbose", d_->verbose,
                    "If true, write status messages to the terminal showing "
                    "optimization progress at each iteration");
  config->set_value("num_threads", o.num_threads,
                    "Number of threads to use");
  config->set_value("num_linear_solver_threads", o.num_linear_solver_threads,
                    "Number of threads to use in the linear solver");
  config->set_value("max_num_iterations", o.max_num_iterations,
                    "Maximum number of iteration of allow");
  config->set_value("function_tolerance", o.function_tolerance,
                    "Solver terminates if relative cost change is below this "
                    "tolerance");
  config->set_value("gradient_tolerance", o.gradient_tolerance,
                    "Solver terminates if the maximum gradient is below this "
                    "tolerance");
  config->set_value("parameter_tolerance", o.parameter_tolerance,
                    "Solver terminates if the relative change in parameters "
                    "is below this tolerance");
  config->set_value("linear_solver_type", o.linear_solver_type,
                    "Linear solver to use."
                    + ceres_options< ::ceres::LinearSolverType >());
  config->set_value("preconditioner_type", o.preconditioner_type,
                    "Preconditioner to use."
                    + ceres_options< ::ceres::PreconditionerType >());
  config->set_value("trust_region_strategy_type", o.trust_region_strategy_type,
                    "Trust region step compution algorithm used by Ceres."
                    + ceres_options< ::ceres::TrustRegionStrategyType >());
  config->set_value("dogleg_type", o.dogleg_type,
                    "Dogleg strategy to use."
                    + ceres_options< ::ceres::DoglegType >());
  config->set_value("loss_function_type", d_->loss_function_type,
                    "Robust loss function type to use."
                    + ceres_options< ceres::LossFunctionType >());
  config->set_value("loss_function_scale", d_->loss_function_scale,
                    "Robust loss function scale factor.");
  config->set_value("optimize_focal_length", d_->optimize_focal_length,
                    "Include focal length parameters in bundle adjustment.");
  config->set_value("optimize_aspect_ratio", d_->optimize_aspect_ratio,
                    "Include aspect ratio parameters in bundle adjustment.");
  config->set_value("optimize_principal_point", d_->optimize_principal_point,
                    "Include principal point parameters in bundle adjustment.");
  config->set_value("optimize_skew", d_->optimize_skew,
                    "Include skew parameters in bundle adjustment.");
  config->set_value("lens_distortion_type", d_->lens_distortion_type,
                    "Lens distortion model to use."
                    + ceres_options< ceres::LensDistortionType >());
  config->set_value("optimize_dist_k1", d_->optimize_dist_k1,
                    "Include radial lens distortion parameter k1 in "
                    "bundle adjustment.");
  config->set_value("optimize_dist_k2", d_->optimize_dist_k2,
                    "Include radial lens distortion parameter k2 in "
                    "bundle adjustment.");
  config->set_value("optimize_dist_k3", d_->optimize_dist_k3,
                    "Include radial lens distortion parameter k3 in "
                    "bundle adjustment.");
  config->set_value("optimize_dist_p1_p2", d_->optimize_dist_p1_p2,
                    "Include tangential lens distortion parameters "
                    "p1 and p2 in bundle adjustment.");
  config->set_value("optimize_dist_k4_k5_k6", d_->optimize_dist_k4_k5_k6,
                    "Include radial lens distortion parameters "
                    "k4, k5, and k6 in bundle adjustment.");
  config->set_value("camera_intrinsic_share_type", d_->camera_intrinsic_share_type,
                    "Determines how to share intrinsics across cameras.\n"
                    "AUTO shares intrinsics between cameras with a common camera_intrinsic_sptr\n"
                    "COMMON enforces that all cameras share common intrinsics\n"
                    "UNIQUE enforces that each camera has its own intrinsics parameters."
                    + ceres_options< ceres::CameraIntrinsicShareType >());
  return config;
}


/// Set this algorithm's properties via a config block
void
bundle_adjust
::set_configuration(config_block_sptr in_config)
{
  ::ceres::Solver::Options& o = d_->options;
  // Starting with our generated config_block to ensure that assumed values are present
  // An alternative is to check for key presence before performing a get_value() call.
  config_block_sptr config = this->get_configuration();
  config->merge_config(in_config);

  d_->verbose = config->get_value<bool>("verbose",
                                        d_->verbose);
  o.minimizer_progress_to_stdout = d_->verbose;
  o.logging_type = d_->verbose ? ::ceres::PER_MINIMIZER_ITERATION
                               : ::ceres::SILENT;
  o.num_threads = config->get_value<int>("num_threads", o.num_threads);
  o.num_linear_solver_threads
      = config->get_value<int>("num_linear_solver_threads",
                               o.num_linear_solver_threads);
  o.max_num_iterations = config->get_value<int>("max_num_iterations",
                                                o.max_num_iterations);
  o.function_tolerance = config->get_value<double>("function_tolerance",
                                                   o.function_tolerance);
  o.gradient_tolerance = config->get_value<double>("gradient_tolerance",
                                                   o.gradient_tolerance);
  o.parameter_tolerance = config->get_value<double>("parameter_tolerance",
                                                    o.parameter_tolerance);
  typedef ::ceres::LinearSolverType cls_t;
  o.linear_solver_type = config->get_value<cls_t>("linear_solver_type",
                                                  o.linear_solver_type);
  typedef ::ceres::PreconditionerType cpc_t;
  o.preconditioner_type = config->get_value<cpc_t>("preconditioner_type",
                                                   o.preconditioner_type);
  typedef ::ceres::TrustRegionStrategyType trs_t;
  o.trust_region_strategy_type
      = config->get_value<trs_t>("trust_region_strategy_type",
                                 o.trust_region_strategy_type);
  typedef ::ceres::DoglegType cdl_t;
  o.dogleg_type = config->get_value<cdl_t>("dogleg_type",
                                           o.dogleg_type);
  typedef ceres::LossFunctionType clf_t;
  d_->loss_function_type = config->get_value<clf_t>("loss_function_type",
                                                    d_->loss_function_type);
  d_->loss_function_scale = config->get_value<double>("loss_function_scale",
                                                      d_->loss_function_scale);
  d_->optimize_focal_length = config->get_value<bool>("optimize_focal_length",
                                                      d_->optimize_focal_length);
  d_->optimize_aspect_ratio = config->get_value<bool>("optimize_aspect_ratio",
                                                      d_->optimize_aspect_ratio);
  d_->optimize_principal_point = config->get_value<bool>("optimize_principal_point",
                                                         d_->optimize_principal_point);
  d_->optimize_skew = config->get_value<bool>("optimize_skew",
                                              d_->optimize_skew);
  typedef ceres::LensDistortionType cld_t;
  d_->lens_distortion_type = config->get_value<cld_t>("lens_distortion_type",
                                                      d_->lens_distortion_type);
  d_->optimize_dist_k1 = config->get_value<bool>("optimize_dist_k1",
                                                 d_->optimize_dist_k1);
  d_->optimize_dist_k2 = config->get_value<bool>("optimize_dist_k2",
                                                 d_->optimize_dist_k2);
  d_->optimize_dist_k3 = config->get_value<bool>("optimize_dist_k3",
                                                 d_->optimize_dist_k3);
  d_->optimize_dist_p1_p2 = config->get_value<bool>("optimize_dist_p1_p2",
                                                    d_->optimize_dist_p1_p2);
  d_->optimize_dist_k4_k5_k6 = config->get_value<bool>("optimize_dist_k4_k5_k6",
                                                       d_->optimize_dist_k4_k5_k6);
  typedef ceres::CameraIntrinsicShareType ccis_t;
  d_->camera_intrinsic_share_type =
      config->get_value<ccis_t>("camera_intrinsic_share_type",
                                d_->camera_intrinsic_share_type);
}


/// Check that the algorithm's currently configuration is valid
bool
bundle_adjust
::check_configuration(config_block_sptr config) const
{
  std::string msg;
  if( !d_->options.IsValid(&msg) )
  {
    LOG_ERROR( d_->m_logger, msg);
    return false;
  }
  return true;
}


/// Optimize the camera and landmark parameters given a set of tracks
void
bundle_adjust
::optimize(camera_map_sptr& cameras,
           landmark_map_sptr& landmarks,
           track_set_sptr tracks) const
{
  if( !cameras || !landmarks || !tracks )
  {
    // TODO throw an exception for missing input data
    return;
  }
  typedef camera_map::map_camera_t map_camera_t;
  typedef landmark_map::map_landmark_t map_landmark_t;

#define MAPTK_SBA_TIMED(msg, code) \
  do \
  { \
    boost::timer::cpu_timer t; \
    if (d_->verbose) \
    { \
      std::cerr << msg << " ... " << std::endl; \
    } \
    code \
    if (d_->verbose) \
    { \
      cpu_times elapsed = t.elapsed(); \
      /* convert nanosecond to seconds */ \
      double secs = static_cast<double>(elapsed.system + elapsed.user) * 0.000000001; \
      std::cerr << "--> " << secs << " s CPU" << std::endl; \
    } \
  } while(false)

  // extract data from containers
  map_camera_t cams = cameras->cameras();
  map_landmark_t lms = landmarks->landmarks();
  std::vector<track_sptr> trks = tracks->tracks();

  // Extract the landmark locations into a mutable map
  typedef std::map<track_id_t, std::vector<double> > lm_param_map_t;
  lm_param_map_t landmark_params;
  VITAL_FOREACH(const map_landmark_t::value_type& lm, lms)
  {
    vector_3d loc = lm.second->loc();
    landmark_params[lm.first] = std::vector<double>(loc.data(), loc.data()+3);
  }

  // Extract the camera parameters into a mutable map
  //
  typedef std::map<frame_id_t, std::vector<double> > cam_param_map_t;
  cam_param_map_t camera_params;
  // We need maps from both frame number and intrinsics_sptr to the
  // index of the camera parameters.  This way we can vary how each
  // frame maps to a set of intrinsic parameters based on config params.
  typedef std::map<camera_intrinsics_sptr, unsigned int> cam_intrin_map_t;
  cam_intrin_map_t camera_intr_map;
  typedef std::map<frame_id_t, unsigned int> frame_to_intrin_map_t;
  frame_to_intrin_map_t frame_to_intr_map;
  // vector of unique camera intrinsic parameters
  std::vector<std::vector<double> > camera_intr_params;
  // number of lens distortion parameters in the selected model
  const unsigned int ndp = num_distortion_params(d_->lens_distortion_type);
  std::vector<double> intrinsic_params(5 + ndp, 0.0);
  VITAL_FOREACH(const map_camera_t::value_type& c, cams)
  {
    vector_3d rot = c.second->rotation().rodrigues();
    vector_3d center = c.second->center();
    std::vector<double> params(6);
    std::copy(rot.data(), rot.data()+3, params.begin());
    std::copy(center.data(), center.data()+3, params.begin()+3);
    camera_intrinsics_sptr K = c.second->intrinsics();
    camera_params[c.first] = params;

    // add a new set of intrisic parameter if one of the following:
    // - we are forcing unique intrinsics for each camera
    // - we are forcing common intrinsics and this is the first frame
    // - we are auto detecting shared intrinisics and this is a new sptr
    if( d_->camera_intrinsic_share_type == FORCE_UNIQUE_INTRINSICS
        || ( d_->camera_intrinsic_share_type == FORCE_COMMON_INTRINSICS
             && camera_intr_params.empty() )
        || camera_intr_map.count(K) == 0 )
    {
      intrinsic_params[0] = K->focal_length();
      intrinsic_params[1] = K->principal_point().x();
      intrinsic_params[2] = K->principal_point().y();
      intrinsic_params[3] = K->aspect_ratio();
      intrinsic_params[4] = K->skew();
      const std::vector<double> d = K->dist_coeffs();
      // copy the intersection of parameters provided in K
      // and those that are supported by the requested model type
      unsigned int num_dp = std::min(ndp, static_cast<unsigned int>(d.size()));
      if( num_dp > 0 )
      {
        std::copy(d.begin(), d.begin()+num_dp, &intrinsic_params[5]);
      }
      // update the maps with the index of this new parameter vector
      camera_intr_map[K] = camera_intr_params.size();
      frame_to_intr_map[c.first] = camera_intr_params.size();
      // add the parameter vector
      camera_intr_params.push_back(intrinsic_params);
    }
    else if( d_->camera_intrinsic_share_type == FORCE_COMMON_INTRINSICS )
    {
      // map to the first parameter vector
      frame_to_intr_map[c.first] = 0;
    }
    else
    {
      // map to a previously seen parameter vector
      frame_to_intr_map[c.first] = camera_intr_map[K];
    }
  }

  // the Ceres solver problem
  ::ceres::Problem problem;

  // enumerate the intrinsics held constant
  std::vector<int> constant_intrinsics;
  if (!d_->optimize_focal_length)
  {
    constant_intrinsics.push_back(0);
  }
  if (!d_->optimize_principal_point)
  {
    constant_intrinsics.push_back(1);
    constant_intrinsics.push_back(2);
  }
  if (!d_->optimize_aspect_ratio)
  {
    constant_intrinsics.push_back(3);
  }
  if (!d_->optimize_skew)
  {
    constant_intrinsics.push_back(4);
  }
  if (!d_->optimize_dist_k1 && ndp > 0)
  {
    constant_intrinsics.push_back(5);
  }
  if (!d_->optimize_dist_k2 && ndp > 1)
  {
    constant_intrinsics.push_back(6);
  }
  if (!d_->optimize_dist_p1_p2 && ndp > 3)
  {
    constant_intrinsics.push_back(7);
    constant_intrinsics.push_back(8);
  }
  if (!d_->optimize_dist_k3 && ndp > 4)
  {
    constant_intrinsics.push_back(9);
  }
  if (!d_->optimize_dist_k4_k5_k6 && ndp > 7)
  {
    constant_intrinsics.push_back(10);
    constant_intrinsics.push_back(11);
    constant_intrinsics.push_back(12);
  }

  // Create the loss function to use
  ::ceres::LossFunction* loss_func
      = LossFunctionFactory(d_->loss_function_type,
                            d_->loss_function_scale);
  bool loss_func_used = false;

  // Add the residuals for each relevant observation
  VITAL_FOREACH(const track_sptr& t, trks)
  {
    const track_id_t id = t->id();
    lm_param_map_t::iterator lm_itr = landmark_params.find(id);
    // skip this track if the landmark is not in the set to optimize
    if( lm_itr == landmark_params.end() )
    {
      continue;
    }

    for(track::history_const_itr ts = t->begin(); ts != t->end(); ++ts)
    {
      cam_param_map_t::iterator cam_itr = camera_params.find(ts->frame_id);
      if( cam_itr == camera_params.end() )
      {
        continue;
      }
      unsigned intr_idx = frame_to_intr_map[ts->frame_id];
      double * intr_params_ptr = &camera_intr_params[intr_idx][0];
      vector_2d pt = ts->feat->loc();
      problem.AddResidualBlock(create_cost_func(d_->lens_distortion_type,
                                                pt.x(), pt.y()),
                               loss_func,
                               intr_params_ptr,
                               &cam_itr->second[0],
                               &lm_itr->second[0]);
      loss_func_used = true;
    }
  }
  VITAL_FOREACH(std::vector<double>& cip, camera_intr_params)
  {
    // apply the constraints
    if (constant_intrinsics.size() > 4 + ndp)
    {
      // set all parameters in the block constant
      problem.SetParameterBlockConstant(&cip[0]);
    }
    else if (!constant_intrinsics.empty())
    {
      // set a subset of parameters in the block constant
      problem.SetParameterization(&cip[0],
          new ::ceres::SubsetParameterization(5 + ndp, constant_intrinsics));
    }
  }

  // If the loss function was added to a residual block, ownership was
  // transfered.  If not then we need to delete it.
  if(loss_func && !loss_func_used)
  {
    delete loss_func;
  }

  ::ceres::Solver::Summary summary;
  ::ceres::Solve(d_->options, &problem, &summary);
  std::cout << summary.FullReport() << "\n";

  // Update the landmarks with the optimized values
  VITAL_FOREACH(const lm_param_map_t::value_type& lmp, landmark_params)
  {
    auto& lmi = lms[lmp.first];
    auto updated_lm = std::make_shared<landmark_d>(*lmi);
    updated_lm->set_loc(Eigen::Map<const vector_3d>(&lmp.second[0]));
    lmi = updated_lm;
  }

  // Update the camera intrinics with optimized values
  std::vector<camera_intrinsics_sptr> updated_intr;
  VITAL_FOREACH(const std::vector<double>& cip, camera_intr_params)
  {
    simple_camera_intrinsics* K = new simple_camera_intrinsics();
    K->set_focal_length(cip[0]);
    vector_2d pp((Eigen::Map<const vector_2d>(&cip[1])));
    K->set_principal_point(pp);
    K->set_aspect_ratio(cip[3]);
    K->set_skew(cip[4]);
    if( ndp > 0 )
    {
      Eigen::VectorXd dc(ndp);
      std::copy(&cip[5], &cip[5]+ndp, dc.data());
      K->set_dist_coeffs(dc);
    }
    updated_intr.push_back(camera_intrinsics_sptr(K));
  }

  // Update the cameras with the optimized values
  VITAL_FOREACH(const cam_param_map_t::value_type& cp, camera_params)
  {
    vector_3d center = Eigen::Map<const vector_3d>(&cp.second[3]);
    rotation_d rot(vector_3d(Eigen::Map<const vector_3d>(&cp.second[0])));

    // look-up updated intrinsics
    unsigned int intr_idx = frame_to_intr_map[cp.first];
    camera_intrinsics_sptr K = updated_intr[intr_idx];
    cams[cp.first] = camera_sptr(new simple_camera(center, rot, K));
  }


  cameras = camera_map_sptr(new simple_camera_map(cams));
  landmarks = landmark_map_sptr(new simple_landmark_map(lms));

#undef MAPTK_SBA_TIMED
}


} // end namespace ceres

} // end namespace maptk
} // end namespace kwiver
