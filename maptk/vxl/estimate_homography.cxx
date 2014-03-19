/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief VXL homography estimation algorithm implementation
 */


#include <maptk/vxl/estimate_homography.h>

#include <boost/foreach.hpp>

#include <vnl/vnl_double_3.h>
#include <vnl/vnl_double_3x3.h>
#include <rrel/rrel_homography2d_est.h>
#include <rrel/rrel_irls.h>
#include <rrel/rrel_trunc_quad_obj.h>
#include <rrel/rrel_ran_sam_search.h>

namespace maptk
{

namespace vxl
{


/// Estimate a homography matrix from corresponding points
homography
estimate_homography
::estimate(const std::vector<vector_2d>& pts1,
           const std::vector<vector_2d>& pts2,
           std::vector<bool>& inliers,
           double inlier_scale) const
{
  if (pts1.size() < 4 || pts2.size() < 4)
  {
    std::cerr << "Not enough points to estimate a homography" <<std::endl;
    return homography(0.0);
  }

  std::vector< vnl_vector<double> > from_pts, to_pts;
  BOOST_FOREACH(const vector_2d& v, pts1)
  {
    from_pts.push_back(vnl_double_3(v.x(), v.y(), 1.0));
  }
  BOOST_FOREACH(const vector_2d& v, pts2)
  {
    to_pts.push_back(vnl_double_3(v.x(), v.y(), 1.0));
  }

  // Step 1: estimate the homography using sampling.  This will allow
  // a good rejection of outliers.
  rrel_homography2d_est hg( from_pts, to_pts );
  hg.set_prior_scale(inlier_scale);

  rrel_trunc_quad_obj msac;
  // TODO expose these parameters
  rrel_ran_sam_search ransam( 42 );
  ransam.set_sampling_params(0.80);
  ransam.set_trace_level(0);

  bool result = ransam.estimate( &hg, &msac );

  std::vector<double> residuals = ransam.residuals();

  if ( ! result )
  {
    std::cerr << "MSAC failed!!" << std::endl;
    return homography(0.0);
  }

  // Step 2: refine the estimate using weighted least squares.  This
  // will allow us to estimate a homography that does not exactly
  // fit 4 points, which will be a better estimate.  The ransam
  // estimate from step 2 would have gotten us close enough to the
  // correct solution for IRLS to work.
  rrel_irls irls;
  irls.set_no_scale_est();
  irls.initialize_scale(inlier_scale);
  irls.initialize_params( ransam.params() );
  bool result2 = irls.estimate( &hg, &msac );

  vnl_double_3x3 m;
  if( ! result2 )
  {
    // if the IRLS fails, fall back to the ransam estimate.
    std::cerr << "IRLS failed" << std::endl;
    hg.params_to_homog( ransam.params(), m.as_ref().non_const() );
  }
  else
  {
    hg.params_to_homog( irls.params(), m.as_ref().non_const() );
    hg.compute_residuals(irls.params(), residuals);
  }

  inliers.clear();
  BOOST_FOREACH(const double& r, residuals)
  {
    inliers.push_back(r < inlier_scale);
  }

  return homography(m.data_block());
}


} // end namespace vxl

} // end namespace maptk
