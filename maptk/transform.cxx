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
 * \brief Implementation of maptk::transform functions to apply
 * similarity transformations
 */

#include "transform.h"
#include <boost/foreach.hpp>
#include <Eigen/Geometry>

#include <maptk/camera.h>

namespace kwiver {
namespace maptk {


/// Transform the camera by applying a similarity transformation in place
template <typename T>
void
transform_inplace(const vital::similarity_<T>& xform,
                  vital::camera_<T>& cam)
{
  cam.set_center( xform * cam.get_center() );
  cam.set_rotation( cam.get_rotation() * xform.rotation().inverse() );
  cam.set_center_covar( transform(cam.get_center_covar(), xform) );
}


/// Transform the camera by applying a similarity transformation in place
template <typename T>
void
transform_inplace(const vital::similarity_<T>& xform,
                  maptk::camera_<T>& cam)
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
  if( vital::camera_d* vcam = dynamic_cast<vital::camera_d*>(cam.get()) )
  {
    transform_inplace(xform, *vcam);
  }
  else if( vital::camera_f* vcam = dynamic_cast<vital::camera_f*>(cam.get()) )
  {
    transform_inplace(vital::similarity_f(xform), *vcam);
  }
  else if( maptk::camera_d* mcam = dynamic_cast<maptk::camera_d*>(cam.get()) )
  {
    transform_inplace(xform, *mcam);
  }
  else if( maptk::camera_f* mcam = dynamic_cast<maptk::camera_f*>(cam.get()) )
  {
    transform_inplace(vital::similarity_f(xform), *mcam);
  }
  else
  {
    maptk::camera_d* new_cam =
        new maptk::camera_d( xform * cam->center(),
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
  BOOST_FOREACH(vital::camera_map::map_camera_t::value_type& p, cam_map)
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
    vital::landmark_d* new_lm =
        new vital::landmark_d( xform * lm->loc(),
                               lm->scale() * xform.scale() );
    new_lm->set_covar( transform(lm->covar(), xform) );
    lm = vital::landmark_sptr( new_lm );
  }
  return lm;
}


/// construct a transformed map of landmarks by applying a similarity transformation
vital::landmark_map_sptr transform(vital::landmark_map_sptr landmarks,
                            const vital::similarity_d& xform)
{
  vital::landmark_map::map_landmark_t lm_map = landmarks->landmarks();
  BOOST_FOREACH(vital::landmark_map::map_landmark_t::value_type& p, lm_map)
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
  BOOST_FOREACH(const lm_map_t::value_type& p, landmarks->landmarks())
  {
    vital::vector_3d c = p.second->loc();
    center += c;
    s += c.dot(c);
  }
  center /= static_cast<double>(landmarks->size());
  s /= landmarks->size();
  s -= center.dot(center);
  s = 1.0/std::sqrt(s);

  // find the average look direction and average up direction
  vital::vector_3d cam_center(0,0,0);
  vital::vector_3d cam_up(0,0,0);
  typedef vital::camera_map::map_camera_t cam_map_t;
  BOOST_FOREACH(const cam_map_t::value_type& p, cameras->cameras())
  {
    cam_center += p.second->center();
    cam_up += p.second->rotation().inverse() * vital::vector_3d(0,1,0);
  }
  cam_center /= static_cast<double>(cameras->size());
  cam_center -= center;
  cam_center = cam_center.normalized();
  cam_up = (-cam_up).normalized();
  vital::vector_3d cam_x = cam_up.cross(cam_center).normalized();
  vital::vector_3d cam_y = cam_center.cross(cam_x).normalized();
  vital::matrix_3x3d rot;
  rot.col(0) = cam_x;
  rot.col(1) = cam_y;
  rot.col(2) = cam_center;
  vital::rotation_d R(rot);
  R = R.inverse();

  return vital::similarity_d(s, R, R*(-s*center));
}


/// \cond DoxygenSuppress
#define INSTANTIATE_TRANSFORM(T) \
template MAPTK_LIB_EXPORT vital::covariance_<3,T> \
transform(const vital::covariance_<3,T>& covar, \
          const vital::similarity_<T>& xform); \
template MAPTK_LIB_EXPORT void \
transform_inplace(const vital::similarity_<T>& xform, \
                  vital::camera_<T>& cam); \
template MAPTK_LIB_EXPORT void \
transform_inplace(const vital::similarity_<T>& xform, \
                  maptk::camera_<T>& cam); \
template MAPTK_LIB_EXPORT void \
transform_inplace(const vital::similarity_<T>& xform, \
                  vital::landmark_<T>& cam);

INSTANTIATE_TRANSFORM(double);
INSTANTIATE_TRANSFORM(float);

#undef INSTANTIATE_TRANSFORM
/// \endcond


} // end namespace maptk
} // end namespace kwiver
