/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief Header for maptk::transform functions
 */

#ifndef MAPTK_TRANSFORM_H_
#define MAPTK_TRANSFORM_H_

#include "similarity.h"
#include "covariance.h"
#include "camera_map.h"
#include "landmark_map.h"


namespace maptk
{


/// Transform a 3D covariance matrix with a similarity transformation
/**
 *  This function applies the scale and rotation of a similarity transformation
 *  to a covariance matrix such that the Mahalanobis distance measure between
 *  two points remains unchanged after applying the same transformation to the
 *  points.  That is,
 *       (x1-m1)'*C1*(x1-m1) == (x2-m2)'*C2*(x2-m2)
 *       for x2 = xform*x1 and m2 = xform*m1 and C2 = transform(C1, xform)
 *
 *  \param [in] covar the 3D covariance to transform
 *  \param [in] xform the 3D similarity transformation to apply
 *  \return a 3D covariance transformed by the similarity transformation
 */
template <typename T>
MAPTK_CORE_EXPORT
covariance_<3,T> transform(const covariance_<3,T>& covar,
                           const similarity_<T>& xform);


/// construct a transformed camera by applying a similarity transformation
MAPTK_CORE_EXPORT
camera_sptr transform(camera_sptr cam,
                      const similarity_d& xform);


/// construct a transformed map of cameras by applying a similarity transformation
MAPTK_CORE_EXPORT
camera_map_sptr transform(camera_map_sptr cameras,
                          const similarity_d& xform);


/// construct a transformed landmark by applying a similarity transformation
MAPTK_CORE_EXPORT
landmark_sptr transform(landmark_sptr lm,
                        const similarity_d& xform);


/// construct a transformed map of landmarks by applying a similarity transformation
MAPTK_CORE_EXPORT
landmark_map_sptr transform(landmark_map_sptr landmarks,
                            const similarity_d& xform);

} // end namespace maptk


#endif // MAPTK_TRANSFORM_H_
