/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief Header for evaluation metric functions.
 */

#ifndef MAPTK_METRICS_H_
#define MAPTK_METRICS_H_

#include "core_config.h"

#include "camera.h"
#include "landmark.h"
#include "feature.h"
#include "track.h"
#include <vector>
#include <map>
#include <cmath>

namespace maptk
{


/// Compute the reprojection error vector of lm projected by cam compared to f
/**
 * \param [in] cam is the camera used for projection
 * \param [in] lm is the landmark projected into the camera
 * \param [in] f is the measured feature point location
 * \returns the vector between the projected lm and f in image space
 */
MAPTK_CORE_EXPORT
vector_2d reprojection_error_vec(const camera& cam,
                                 const landmark& lm,
                                 const feature& f);


/// Compute the square reprojection error of lm projected by cam compared to f
/**
 * \param [in] cam is the camera used for projection
 * \param [in] lm is the landmark projected into the camera
 * \param [in] f is the measured feature point location
 * \returns the squared distance between the projected lm and f in image space
 */
inline
double
reprojection_error_sqr(const camera& cam,
                       const landmark& lm,
                       const feature& f)
{
  return reprojection_error_vec(cam, lm, f).magnitude_sqr();
}


/// Compute the reprojection error of lm projected by cam compared to f
/**
 * \param [in] cam is the camera used for projection
 * \param [in] lm is the landmark projected into the camera
 * \param [in] f is the measured feature point location
 * \returns the distance between the projected lm and f in image space
 */
inline
double
reprojection_error(const camera& cam,
                   const landmark& lm,
                   const feature& f)
{
  return std::sqrt(reprojection_error_sqr(cam, lm, f));
}


/// Compute the Root-Mean-Square-Error (RMSE) of the reprojections
/**
 * \param [in] cameras is the map of frames/cameras used for projection
 * \param [in] landmarks is the map ids/landmarks projected into the cameras
 * \param [in] tracks is the set of tracks providing measurements
 * \returns the RMSE between all landmarks projected by all cameras that have
 *          corresponding image measurements provided by the tracks
 */
MAPTK_CORE_EXPORT
double
reprojection_rmse(const std::map<frame_id_t, camera_sptr>& cameras,
                  const std::map<landmark_id_t, landmark_sptr>& landmarks,
                  const std::vector<track_sptr>& tracks);


} // end namespace maptk


#endif // MAPTK_METRICS_H_
