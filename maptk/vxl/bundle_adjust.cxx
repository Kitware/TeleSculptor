/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief Implementation of VXL bundle adjustment algorithm
 */

#include <maptk/vxl/bundle_adjust.h>
#include <maptk/vxl/camera_map.h>
#include <boost/foreach.hpp>
#include <set>
#include <vpgl/algo/vpgl_bundle_adjust.h>

namespace maptk
{

namespace vxl
{

/// Get this algorithm's \link maptk::config_block configuration block \endlink
config_block_sptr
bundle_adjust
::get_configuration() const
{
  // get base config from base class
  config_block_sptr config = maptk::algo::bundle_adjust::get_configuration();

  return config;
}


/// Set this algorithm's properties via a config block
void
bundle_adjust
::set_configuration(config_block_sptr in_config)
{
  // Starting with our generated config_block to ensure that assumed values are present
  // An alternative is to check for key presence before performing a get_value() call.
  config_block_sptr config = this->get_configuration();
  config->merge_config(in_config);
}


/// Check that the algorithm's currently configuration is valid
bool
bundle_adjust
::check_configuration(config_block_sptr config) const
{
  return true;
}


/// Optimize the camera and landmark parameters given a set of tracks
void
bundle_adjust
::optimize(camera_map_sptr& cameras,
           landmark_map_sptr& landmarks,
           track_set_sptr tracks) const
{
  if( !cameras || !landmarks || !tracks )
  {
    // TODO throw an exception for missing input data
    return;
  }
  typedef vxl::camera_map::map_vcam_t map_vcam_t;
  typedef maptk::landmark_map::map_landmark_t map_landmark_t;

  // extract data from containers
  map_vcam_t vcams = camera_map_to_vpgl(*cameras);
  map_landmark_t lms = landmarks->landmarks();
  std::vector<track_sptr> trks = tracks->tracks();

  // find the set of all frame numbers containing a camera and track data
  std::set<track_id_t> lm_ids;
  typedef std::map<frame_id_t, std::set<track_id_t> > id_map_t;
  id_map_t id_map;
  BOOST_FOREACH(const map_vcam_t::value_type& p, vcams)
  {
    const frame_id_t& frame = p.first;
    track_set_sptr ftracks = tracks->active_tracks(static_cast<int>(frame));
    if (! ftracks || ftracks->size() == 0)
    {
      continue;
    }
    std::set<track_id_t> frame_lm_ids;
    BOOST_FOREACH(const track_sptr& t, ftracks->tracks())
    {
      const track_id_t id = t->id();
      // make sure the track id has an associated landmark
      if( lms.find(id) != lms.end() )
      {
        frame_lm_ids.insert(id);
        lm_ids.insert(id);
      }
    }
    if( !frame_lm_ids.empty() )
    {
      id_map[frame] = frame_lm_ids;
    }
  }

  // create a compact set of data to optimize,
  // with mapping back to original indices
  std::vector<track_id_t> lm_id_index;
  std::map<track_id_t, frame_id_t> lm_id_reverse_map;
  std::vector<vgl_point_3d<double> > active_world_pts;
  BOOST_FOREACH(const track_id_t& id, lm_ids)
  {
    lm_id_reverse_map[id] = lm_id_index.size();
    lm_id_index.push_back(id);
    vector_3d pt = lms[id]->loc();
    active_world_pts.push_back(vgl_point_3d<double>(pt.x(), pt.y(), pt.z()));
  }
  std::vector<frame_id_t> cam_id_index;
  std::map<frame_id_t, frame_id_t> cam_id_reverse_map;
  std::vector<vpgl_perspective_camera<double> > active_vcams;
  BOOST_FOREACH(const id_map_t::value_type& p, id_map)
  {
    cam_id_reverse_map[p.first] = cam_id_index.size();
    cam_id_index.push_back(p.first);
    active_vcams.push_back(vcams[p.first]);
  }

  // Construct the camera/landmark visibility matrix
  std::vector<std::vector<bool> >
      mask(active_vcams.size(),
           std::vector<bool>(active_world_pts.size(), false));
  BOOST_FOREACH(const id_map_t::value_type& p, id_map)
  {
    const frame_id_t c_idx = cam_id_reverse_map[p.first];
    std::vector<bool>& mask_row = mask[c_idx];
    BOOST_FOREACH(const track_id_t& lm_idx, p.second)
    {
      mask_row[lm_id_reverse_map[lm_idx]] = true;
    }
  }

  // Populate the vector of observations in the correct order
  std::vector<vgl_point_2d<double> > image_pts;
  for (unsigned int i=0; i<active_vcams.size(); ++i)
  {
    const frame_id_t f_id = cam_id_index[i];
    for (unsigned int j=0; j<active_world_pts.size(); ++j)
    {
      if(mask[i][j])
      {
        const track_id_t t_id = lm_id_index[j];
        track_sptr t;
        BOOST_FOREACH(t, trks)
        {
          if(t->id() == t_id)
          {
            break;
          }
        }
        track::history_const_itr tsi = t->find(f_id);
        vector_2d loc(tsi->feat->loc());
        image_pts.push_back(vgl_point_2d<double>(loc.x(), loc.y()));
      }
    }
  }

  // Run the vpgl bundle adjustment on the selected data
  vpgl_bundle_adjust ba;
  ba.optimize(active_vcams, active_world_pts, image_pts, mask);

  // map optimized results back into maptk structures
  for(unsigned int i=0; i<active_vcams.size(); ++i)
  {
    vcams[cam_id_index[i]] = active_vcams[i];
  }
  for(unsigned int i=0; i<active_world_pts.size(); ++i)
  {
    const vgl_point_3d<double>& pt = active_world_pts[i];
    vector_3d loc(pt.x(), pt.y(), pt.z());
    landmark_sptr lm = lms[lm_id_index[i]];
    if( landmark_d* lmd = dynamic_cast<landmark_d*>(lm.get()) )
    {
      lmd->set_loc(loc);
    }
    else if( landmark_f* lmf = dynamic_cast<landmark_f*>(lm.get()) )
    {
      lmf->set_loc(vector_3f(loc));
    }
  }
  cameras = camera_map_sptr(new camera_map(vcams));
  landmarks = landmark_map_sptr(new simple_landmark_map(lms));
}


} // end namespace vxl

} // end namespace maptk
