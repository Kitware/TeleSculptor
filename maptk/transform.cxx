/*ckwg +29
 * Copyright 2014-2016 by Kitware, Inc.
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
 * \brief Implementation of maptk::transform functions to apply
 * similarity transformations
 */

#include "transform.h"
#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <vital/vital_foreach.h>

namespace kwiver {
namespace maptk {


/// Transform the camera by applying a similarity transformation in place
void
transform_inplace(const vital::similarity_d& xform,
                  vital::simple_camera& cam)
{
  cam.set_center( xform * cam.get_center() );
  cam.set_rotation( cam.get_rotation() * xform.rotation().inverse() );
  cam.set_center_covar( transform(cam.get_center_covar(), xform) );
}


/// Transform the landmark by applying a similarity transformation in place
template <typename T>
void
transform_inplace(const vital::similarity_<T>& xform,
                  vital::landmark_<T>& lm)
{
  lm.set_loc( xform * lm.get_loc() );
  lm.set_scale( lm.get_scale() * xform.scale() );
  lm.set_covar( transform(lm.get_covar(), xform) );
}


/// Transform a 3D covariance matrix with a similarity transformation
template <typename T>
vital::covariance_<3,T> transform(const vital::covariance_<3,T>& covar,
                           const vital::similarity_<T>& xform)
{
  // TODO trasform covariance parameters directly
  // instead of converting to matrix form and back
  Eigen::Matrix<T,3,3> C(covar);
  Eigen::Matrix<T,3,3> sR(xform.rotation());
  sR /= xform.scale();
  C = sR * C * sR.transpose();
  return vital::covariance_<3,T>(C);
}


/// construct a transformed camera by applying a similarity transformation
vital::camera_sptr transform(vital::camera_sptr cam,
                      const vital::similarity_d& xform)
{
  cam = cam->clone();
  if( vital::simple_camera* vcam = dynamic_cast<vital::simple_camera*>(cam.get()) )
  {
    transform_inplace(xform, *vcam);
  }
  else
  {
    vital::simple_camera* new_cam =
        new vital::simple_camera( xform * cam->center(),
                                  cam->rotation() * xform.rotation().inverse(),
                                  cam->intrinsics() );
    new_cam->set_center_covar( transform(cam->center_covar(), xform) );
    cam = vital::camera_sptr( new_cam );
  }
  return cam;
}


/// construct a transformed map of cameras by applying a similarity transformation
vital::camera_map_sptr transform(vital::camera_map_sptr cameras,
                          const vital::similarity_d& xform)
{
  vital::camera_map::map_camera_t cam_map = cameras->cameras();
  VITAL_FOREACH(vital::camera_map::map_camera_t::value_type& p, cam_map)
  {
    p.second = transform(p.second, xform);
  }
  return vital::camera_map_sptr(new vital::simple_camera_map(cam_map));
}


/// construct a transformed landmark by applying a similarity transformation
vital::landmark_sptr transform(vital::landmark_sptr lm,
                        const vital::similarity_d& xform)
{
  lm = lm->clone();
  if( vital::landmark_d* vlm = dynamic_cast<vital::landmark_d*>(lm.get()) )
  {
    transform_inplace(xform, *vlm);
  }
  else if( vital::landmark_f* vlm = dynamic_cast<vital::landmark_f*>(lm.get()) )
  {
    transform_inplace(vital::similarity_f(xform), *vlm);
  }
  else
  {
    auto new_lm = std::make_shared<vital::landmark_d>( *lm );
    new_lm->set_loc( xform * lm->loc() );
    new_lm->set_scale( lm->scale() * xform.scale() );
    new_lm->set_covar( transform(lm->covar(), xform) );
    lm = new_lm;
  }
  return lm;
}


/// construct a transformed map of landmarks by applying a similarity transformation
vital::landmark_map_sptr transform(vital::landmark_map_sptr landmarks,
                            const vital::similarity_d& xform)
{
  vital::landmark_map::map_landmark_t lm_map = landmarks->landmarks();
  VITAL_FOREACH(vital::landmark_map::map_landmark_t::value_type& p, lm_map)
  {
    p.second = transform(p.second, xform);
  }
  return vital::landmark_map_sptr(new vital::simple_landmark_map(lm_map));
}


/// Estimate a canonical coordinate transform for landmarks and cameras
vital::similarity_d
canonical_transform(vital::camera_map_sptr cameras,
                    vital::landmark_map_sptr landmarks)
{
  using namespace maptk;
  // find the centroid and scale of all the landmarks
  typedef vital::landmark_map::map_landmark_t lm_map_t;
  vital::vector_3d center(0,0,0);
  double s=0.0;
  vital::matrix_3x3d covar = vital::matrix_3x3d::Zero();
  VITAL_FOREACH(const lm_map_t::value_type& p, landmarks->landmarks())
  {
    vital::vector_3d pt = p.second->loc();
    center += pt;
    covar += pt * pt.transpose();
    s += pt.dot(pt);
  }
  const double num_lm = static_cast<double>(landmarks->size());
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

  vital::rotation_d R(rot);
  R = R.inverse();
  return vital::similarity_d(s, R, R*(-s*center));
}


/// Compute an approximate Necker reversal of cameras and landmarks
void
necker_reverse(vital::camera_map_sptr& cameras,
               vital::landmark_map_sptr& landmarks)
{
  typedef vital::landmark_map::map_landmark_t lm_map_t;
  typedef vital::camera_map::map_camera_t cam_map_t;

  cam_map_t cams = cameras->cameras();
  lm_map_t lms = landmarks->landmarks();

  // compute the landmark location mean and covariance
  vital::vector_3d lc(0.0, 0.0, 0.0);
  vital::matrix_3x3d covar = vital::matrix_3x3d::Zero();
  VITAL_FOREACH(const lm_map_t::value_type& p, lms)
  {
    vital::vector_3d pt = p.second->loc();
    lc += pt;
    covar += pt * pt.transpose();
  }
  const double num_lm = static_cast<double>(lms.size());
  lc /= num_lm;
  covar /= num_lm;
  covar -= lc * lc.transpose();

  // the mirroring plane will pass through the landmark centeroid (lc)
  // and have a normal vector aligned with the smallest eigenvector of covar
  Eigen::JacobiSVD<vital::matrix_3x3d> svd(covar, Eigen::ComputeFullV);
  vital::vector_3d axis = svd.matrixV().col(2);

  // flip cameras around
  vital::rotation_d Ra180(vital::vector_4d(axis.x(), axis.y(), axis.z(), 0.0));
  vital::rotation_d Rz180(vital::vector_4d(0.0, 0.0, 1.0, 0.0));
  VITAL_FOREACH(cam_map_t::value_type& p, cams)
  {
    vital::simple_camera* flipped = new vital::simple_camera(*p.second);
    // extract the camera center
    const vital::vector_3d cc = flipped->center();
    // extract the camera principal axis
    vital::vector_3d pa = vital::matrix_3x3d(flipped->rotation()).row(2);
    // compute the distance from cc along pa until intersection with
    // the mirroring plane of the points
    const double dist = (lc - cc).dot(axis) / pa.dot(axis);
    // compute the ground point where the principal axis
    // intersects the mirroring plane
    vital::vector_3d gp = cc + dist * pa;
    // rotate the camera center 180 degrees about the mirroring plane normal
    // axis centered at gp, also rotate the camera 180 about its principal axis
    flipped->set_center(Ra180 * (flipped->center() - gp) + gp);
    flipped->set_rotation(Rz180 * flipped->rotation() * Ra180);
    p.second = vital::camera_sptr(flipped);
  }

  // mirror landmark locations about the mirroring plane
  VITAL_FOREACH(lm_map_t::value_type& p, lms)
  {
    vital::vector_3d v = p.second->loc();
    v -= 2.0 * (v - lc).dot(axis) * axis;
    auto new_lm = std::make_shared<vital::landmark_d>(*p.second);
    new_lm->set_loc(v);
    p.second = new_lm;
  }

  cameras = vital::camera_map_sptr(new vital::simple_camera_map(cams));
  landmarks = vital::landmark_map_sptr(new vital::simple_landmark_map(lms));
}


/// \cond DoxygenSuppress
#define INSTANTIATE_TRANSFORM(T) \
template MAPTK_EXPORT vital::covariance_<3,T> \
transform(const vital::covariance_<3,T>& covar, \
          const vital::similarity_<T>& xform); \
template MAPTK_EXPORT void \
transform_inplace(const vital::similarity_<T>& xform, \
                  vital::landmark_<T>& cam);

INSTANTIATE_TRANSFORM(double);
INSTANTIATE_TRANSFORM(float);

#undef INSTANTIATE_TRANSFORM
/// \endcond


} // end namespace maptk
} // end namespace kwiver
