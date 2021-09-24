// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

/**
 * \file
 * \brief Implementation of functions regarding geo reference points files
 */

#include "geo_reference_points_io.h"
#include <vital/exceptions.h>
#include <vital/io/eigen_io.h>
#include <vital/types/geodesy.h>
#include <vital/logger/logger.h>

#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

namespace kwiver {
namespace maptk {

/// Load landmarks and feature tracks from reference points file
void load_reference_file(vital::path_t const& reference_file,
                         vital::local_geo_cs & lgcs,
                         vital::landmark_map_sptr & ref_landmarks,
                         vital::feature_track_set_sptr & ref_track_set)
{
  using namespace std;

  kwiver::vital::logger_handle_t logger( kwiver::vital::get_logger( "load_reference_file" ) );

  // Read in file, creating a landmark map and a vector of feature tracks, associated
  // via IDs
  std::ifstream input_stream(reference_file.c_str(), std::fstream::in);
  if (!input_stream)
  {
    throw vital::file_not_found_exception(reference_file, "Could not open reference points file!");
  }

  // pre-allocated vars for loop
  vital::landmark_id_t cur_id = 1;
  vital::frame_id_t frm;
  vital::vector_2d feat_loc;
  vital::vector_3d vec(0,0,0);

  // used to stream file lines into data types
  std::istringstream ss;

  vital::landmark_map::map_landmark_t reference_lms;
  std::vector<vital::track_sptr> reference_tracks;

  // Mean position of all landmarks.
  vital::vector_3d mean(0,0,0);

  // If the origin is invalid then use the reference points to compute a new origin
  const bool set_lgcs_origin = lgcs.origin().is_empty();
  int crs = lgcs.origin().crs();

  // TODO: put in try-catch around >>'s in case we have an ill-formatted file,
  // or there's a parse error
  LOG_INFO(logger, "Reading ground control points from file: " << reference_file);
  for (std::string line; std::getline(input_stream, line);)
  {
    ss.clear();
    ss.str(line);

    // input landmarks are given in lon/lat/alt format (ignoring alt for now)
    ss >> vec;

    vital::vector_2d lon_lat(vec.x(), vec.y());
    vital::geo_point gp(lon_lat, vital::SRID::lat_lon_WGS84);
    if ( lgcs.origin().is_empty() )
    {
      auto zone = vital::utm_ups_zone( lon_lat );
      crs = (zone.north ? vital::SRID::UTM_WGS84_north : vital::SRID::UTM_WGS84_south) + zone.number;
      LOG_DEBUG(logger, "lgcs origin zone: " << zone.number );
      lgcs.set_origin( vital::geo_point( gp.location( crs ), crs ) );
    }
    vital::vector_3d utm = gp.location(crs);

    vec[0] = utm.x();
    vec[1] = utm.y();
    mean += vec;

    reference_lms[cur_id] = vital::landmark_sptr(new vital::landmark_d(vec));

    // while there's still input left, read in track states
    vital::track_sptr lm_track = vital::track::create();
    lm_track->set_id(static_cast<vital::track_id_t>(cur_id));
    while (ss.peek() != std::char_traits<char>::eof())
    {
      ss >> frm;
      ss >> feat_loc;
      auto fts = std::make_shared<vital::feature_track_state>(frm,
                       vital::feature_sptr(new vital::feature_d(feat_loc)),
                       vital::descriptor_sptr());
      lm_track->append(fts);
    }
    reference_tracks.push_back(lm_track);

    ++cur_id;
  }
  LOG_INFO(logger, "Loaded "<< reference_tracks.size() <<" ground control points");

  if (set_lgcs_origin)
  {
    // Initialize lgcs center
    mean /= static_cast<double>(reference_lms.size());
    lgcs.set_origin( vital::geo_point( mean, crs ) );
    LOG_DEBUG(logger, "mean position (lgcs origin): " << mean.transpose());
  }

  // Scan through reference landmarks, adjusting their location by the lgcs
  // origin.
  LOG_INFO(logger, "transforming ground control points to local coordinates");
  for(vital::landmark_map::map_landmark_t::value_type & p : reference_lms)
  {
    auto loc = p.second->loc() - lgcs.origin().location();
    dynamic_cast<vital::landmark_d*>(p.second.get())->set_loc(loc);
  }

  ref_landmarks = std::make_shared<vital::simple_landmark_map>(reference_lms);
  ref_track_set = std::make_shared<vital::feature_track_set>(reference_tracks);
}

} // end namespace maptk
} // end namespace kwiver
