/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include "transform.h"
#include <boost/foreach.hpp>

/**
 * \file
 * \brief Implementation of maptk::transform functions to apply
 * similarity transformations
 */


namespace maptk
{

/// Transform a 3D covariance matrix with a similarity transformation
template <typename T>
covariance_<3,T> transform(const covariance_<3,T>& covar,
                           const similarity_<T>& xform)
{
  // TODO trasform covariance parameters directly
  // instead of converting to matrix form and back
  matrix_<3,3,T> C(covar);
  matrix_<3,3,T> sR(xform.rotation());
  sR /= xform.scale();
  C = sR * C * sR.transpose();
  return covariance_<3,T>(C);
}


/// construct a transformed camera by applying a similarity transformation
camera_sptr transform(camera_sptr cam,
                      const similarity_d& xform)
{
  cam = cam->clone();
  cam->transform(xform);
  return cam;
}


/// construct a transformed map of cameras by applying a similarity transformation
camera_map_sptr transform(camera_map_sptr cameras,
                          const similarity_d& xform)
{
  camera_map::map_camera_t cam_map = cameras->cameras();
  BOOST_FOREACH(camera_map::map_camera_t::value_type& p, cam_map)
  {
    p.second = transform(p.second, xform);
  }
  return camera_map_sptr(new simple_camera_map(cam_map));
}


/// construct a transformed landmark by applying a similarity transformation
MAPTK_CORE_EXPORT
landmark_sptr transform(landmark_sptr lm,
                        const similarity_d& xform)
{
  lm = lm->clone();
  lm->transform(xform);
  return lm;
}


/// construct a transformed map of landmarks by applying a similarity transformation
landmark_map_sptr transform(landmark_map_sptr landmarks,
                            const similarity_d& xform)
{
  landmark_map::map_landmark_t lm_map = landmarks->landmarks();
  BOOST_FOREACH(landmark_map::map_landmark_t::value_type& p, lm_map)
  {
    p.second = transform(p.second, xform);
  }
  return landmark_map_sptr(new simple_landmark_map(lm_map));
}


#define INSTANTIATE_TRANSFORM(T) \
template MAPTK_CORE_EXPORT covariance_<3,T> transform(const covariance_<3,T>& covar, \
                                                      const similarity_<T>& xform);

INSTANTIATE_TRANSFORM(double);
INSTANTIATE_TRANSFORM(float);

#undef INSTANTIATE_TRANSFORM
} // end namespace maptk
