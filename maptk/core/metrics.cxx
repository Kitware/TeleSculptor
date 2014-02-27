/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief Implementation of evaluation metric functions.
 */

#include "metrics.h"
#include <boost/foreach.hpp>


namespace maptk
{


/// Compute the reprojection error vector of lm projected by cam compared to f
vector_2d
reprojection_error_vec(const camera& cam,
                       const landmark& lm,
                       const feature& f)
{
  vector_2d pt;
  if (const camera_d* camd = dynamic_cast<const camera_d*>(&cam))
  {
    pt = camd->project(lm.loc());
  }
  else if (const camera_f* camf = dynamic_cast<const camera_f*>(&cam))
  {
    pt = vector_2d(camf->project(vector_3f(lm.loc())));
  }
  return pt - f.loc();
}


/// Compute the Root-Mean-Square-Error (RMSE) of the reprojections
double
reprojection_rmse(const std::map<frame_id_t, camera_sptr>& cameras,
                  const std::map<landmark_id_t, landmark_sptr>& landmarks,
                  const std::vector<track_sptr>& tracks)
{
  typedef std::map<landmark_id_t, landmark_sptr>::const_iterator lm_map_itr_t;
  typedef std::map<frame_id_t, camera_sptr>::const_iterator cam_map_itr_t;
  double error_sum = 0.0;
  unsigned num_obs = 0;
  BOOST_FOREACH(const track_sptr& t, tracks)
  {
    lm_map_itr_t lmi = landmarks.find(t->id());
    if (lmi == landmarks.end() || !lmi->second)
    {
      // no landmark corresponding to this track
      continue;
    }
    const landmark& lm = *lmi->second;
    for( track::history_const_itr tsi = t->begin(); tsi != t->end(); ++tsi)
    {
      if (!tsi->feat)
      {
        // no feature for this track state.
        continue;
      }
      const feature& feat = *tsi->feat;
      cam_map_itr_t ci = cameras.find(tsi->frame_id);
      if (ci == cameras.end() || !ci->second)
      {
        // no camera corresponding to this track state
        continue;
      }
      const camera& cam = *ci->second;
      error_sum += reprojection_error_sqr(cam, lm, feat);
      ++num_obs;
    }
  }
  return std::sqrt(error_sum / num_obs);
}

} // end namespace maptk
