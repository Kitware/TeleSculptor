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
  vector_2d pt(0.0, 0.0);
  if (const camera_d* camd = dynamic_cast<const camera_d*>(&cam))
  {
    pt = camd->project(lm.loc());
  }
  else if (const camera_f* camf = dynamic_cast<const camera_f*>(&cam))
  {
    pt = camf->project(lm.loc().cast<float>()).cast<double>();
  }
  return pt - f.loc();
}


/// Compute a vector of all reprojection errors in the data
std::vector<double>
reprojection_errors(const std::map<frame_id_t, camera_sptr>& cameras,
                    const std::map<landmark_id_t, landmark_sptr>& landmarks,
                    const std::vector<track_sptr>& tracks)
{
  typedef std::map<landmark_id_t, landmark_sptr>::const_iterator lm_map_itr_t;
  typedef std::map<frame_id_t, camera_sptr>::const_iterator cam_map_itr_t;
  std::vector<double> errors;
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
      errors.push_back(reprojection_error(cam, lm, feat));
    }
  }
  return errors;
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


/// Compute the median of the reprojection errors
double
reprojection_median_error(const std::map<frame_id_t, camera_sptr>& cameras,
                          const std::map<landmark_id_t, landmark_sptr>& landmarks,
                          const std::vector<track_sptr>& tracks)
{
  std::vector<double> errors = reprojection_errors(cameras, landmarks, tracks);
  std::nth_element(errors.begin(),
                   errors.begin() + errors.size()/2,
                   errors.end());
  return errors[errors.size()/2];
}


} // end namespace maptk
