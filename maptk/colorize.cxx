/*ckwg +29
 * Copyright 2016 by Kitware, Inc.
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
 * \brief Implementation of maptk::colorize functions to extract/compute colors
 */

#include "colorize.h"

#include <vital/vital_foreach.h>

namespace kwiver {
namespace maptk {


/// Extract feature colors from a frame image
vital::track_set_sptr
extract_feature_colors(
  vital::track_set const& tracks,
  vital::image_container const& image,
  vital::frame_id_t frame_id)
{
  auto const& image_data = image.get_image();

  auto tracks_copy = tracks.tracks();
  VITAL_FOREACH (auto& track, tracks_copy)
  {
    auto const si = track->find(frame_id);
    if (si != track->end())
    {
      auto const new_track = std::make_shared<vital::track>();
      new_track->set_id(track->id());

      VITAL_FOREACH (auto const& state, *track)
      {
        if (state.frame_id == frame_id)
        {
          auto new_state = vital::track::track_state{state};

          auto const feat = std::make_shared<vital::feature_d>(*state.feat);
          auto const& loc = feat->get_loc();
          feat->set_color(image_data.at(static_cast<unsigned>(loc[0]),
                                        static_cast<unsigned>(loc[1])));

          new_state.feat = feat;

          new_track->append(new_state);
        }
        else
        {
          new_track->append(state);
        }
      }

      track = new_track;
    }
  }

  return std::make_shared<vital::simple_track_set>(tracks_copy);
}


/// Compute colors for landmarks
vital::landmark_map_sptr compute_landmark_colors(
  vital::landmark_map const& landmarks,
  vital::track_set const& tracks)
{
  auto colored_landmarks = landmarks.landmarks();
  auto const no_such_landmark = colored_landmarks.end();

  VITAL_FOREACH (auto const track, tracks.tracks())
  {
    auto const lmid = static_cast<vital::landmark_id_t>(track->id());
    auto lmi = colored_landmarks.find(lmid);
    if (lmi != no_such_landmark)
    {
      int ra = 0, ga = 0, ba = 0, k = 0; // accumulators
      VITAL_FOREACH (auto const& ts, *track)
      {
        auto const& color = ts.feat->color();
        ra += color.r;
        ga += color.g;
        ba += color.b;
        ++k;
      }

      if (k)
      {
        auto const r = static_cast<unsigned char>(ra / k);
        auto const g = static_cast<unsigned char>(ga / k);
        auto const b = static_cast<unsigned char>(ba / k);

        auto lm = std::make_shared<kwiver::vital::landmark_d>(*(lmi->second));
        lm->set_color({r, g, b});
        lmi->second = lm;
      }
    }
  }

  return std::make_shared<kwiver::vital::simple_landmark_map>(colored_landmarks);
}


} // end namespace maptk
} // end namespace kwiver
