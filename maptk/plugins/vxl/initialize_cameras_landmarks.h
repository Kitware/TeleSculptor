/*ckwg +29
 * Copyright 2014 by Kitware, Inc.
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
 * \brief Header for VXL camera and landmark initialization algorithm
 */

#ifndef MAPTK_PLUGINS_VXL_INITIALIZE_CAMERAS_LANDMARKS_H_
#define MAPTK_PLUGINS_VXL_INITIALIZE_CAMERAS_LANDMARKS_H_

#include <boost/scoped_ptr.hpp>

#include <maptk/algo/initialize_cameras_landmarks.h>
#include <maptk/plugins/vxl/vxl_config.h>


namespace maptk
{

namespace vxl
{

/// A class for initialization of cameras and landmarks using VXL
class MAPTK_VXL_EXPORT initialize_cameras_landmarks
: public algo::algorithm_impl<initialize_cameras_landmarks,
                              algo::initialize_cameras_landmarks>
{
public:
  /// Constructor
  initialize_cameras_landmarks();

  /// Destructor
  virtual ~initialize_cameras_landmarks();

  /// Copy Constructor
  initialize_cameras_landmarks(const initialize_cameras_landmarks& other);

  /// Return the name of this implementation
  std::string impl_name() const { return "vxl"; }

  /// Get this algorithm's \link maptk::config_block configuration block \endlink
  virtual config_block_sptr get_configuration() const;
  /// Set this algorithm's properties via a config block
  virtual void set_configuration(config_block_sptr config);
  /// Check that the algorithm's currently configuration is valid
  virtual bool check_configuration(config_block_sptr config) const;

  /// Initialize the camera and landmark parameters given a set of tracks
  /**
   * The algorithm creates an initial estimate of any missing cameras and
   * landmarks using the available cameras, landmarks, and tracks.
   * It may optionally revise the estimates of exisiting cameras and landmarks.
   *
   * \param [in,out] cameras the cameras to initialize
   * \param [in,out] landmarks the landmarks to initialize
   * \param [in] tracks the tracks to use as constraints
   */
  virtual void
  initialize(camera_map_sptr& cameras,
             landmark_map_sptr& landmarks,
             track_set_sptr tracks) const;

private:
  /// private implementation class
  class priv;
  boost::scoped_ptr<priv> d_;
};


} // end namespace vxl

} // end namespace maptk


#endif // MAPTK_PLUGINS_VXL_INITIALIZE_CAMERAS_LANDMARKS_H_
