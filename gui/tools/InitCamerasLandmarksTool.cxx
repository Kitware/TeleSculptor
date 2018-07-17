/*ckwg +29
 * Copyright 2016-2017 by Kitware, Inc.
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
 *  * Neither the name Kitware, Inc. nor the names of any contributors may be
 *    used to endorse or promote products derived from this software without
 *    specific prior written permission.
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

#include "InitCamerasLandmarksTool.h"
#include "GuiCommon.h"

#include <vital/algo/initialize_cameras_landmarks.h>

#include <QMessageBox>

using kwiver::vital::algo::initialize_cameras_landmarks;
using kwiver::vital::algo::initialize_cameras_landmarks_sptr;

namespace
{
static char const* const BLOCK = "initializer";
static char const* const CONFIG_FILE = "gui_initialize.conf";
}

//-----------------------------------------------------------------------------
class InitCamerasLandmarksToolPrivate
{
public:
  initialize_cameras_landmarks_sptr algorithm;
};

QTE_IMPLEMENT_D_FUNC(InitCamerasLandmarksTool)

//-----------------------------------------------------------------------------
InitCamerasLandmarksTool::InitCamerasLandmarksTool(QObject* parent)
  : AbstractTool(parent), d_ptr(new InitCamerasLandmarksToolPrivate)
{
  this->setText("&Estimate Cameras/Landmarks");
  this->setToolTip(
    "<nobr>Estimate cameras and landmarks from a set of feature tracks</nobr>");
}

//-----------------------------------------------------------------------------
InitCamerasLandmarksTool::~InitCamerasLandmarksTool()
{
}

//-----------------------------------------------------------------------------
AbstractTool::Outputs InitCamerasLandmarksTool::outputs() const
{
  return Cameras | Landmarks;
}

//-----------------------------------------------------------------------------
bool InitCamerasLandmarksTool::execute(QWidget* window)
{
  QTE_D();

  // Check inputs
  if (!this->hasTracks())
  {
    QMessageBox::information(
      window, "Insufficient data",
      "This operation requires feature tracks.");
    return false;
  }

  // Merge project config with default config file
  auto const config = readConfig(CONFIG_FILE);

  // Check configuration
  if (!config)
  {
    QMessageBox::critical(
      window, "Configuration error",
      QString("No configuration data was found. Looking for \"")
      + CONFIG_FILE + "\". Please check your installation.");
    return false;
  }

  config->merge_config(this->data()->config);
  if (!initialize_cameras_landmarks::check_nested_algo_configuration(BLOCK, config))
  {
    QMessageBox::critical(
      window, "Configuration error",
      "An error was found in the algorithm configuration.");
    return false;
  }

  // Create algorithm from configuration
  initialize_cameras_landmarks::set_nested_algo_configuration(
    BLOCK, config, d->algorithm);

  // Set the callback to receive updates
  using std::placeholders::_1;
  using std::placeholders::_2;
  typedef initialize_cameras_landmarks::callback_t callback_t;
  callback_t cb = std::bind(&InitCamerasLandmarksTool::callback_handler, this, _1, _2);
  d->algorithm->set_callback(cb);

  // Hand off to base class
  return AbstractTool::execute(window);
}

//-----------------------------------------------------------------------------
void InitCamerasLandmarksTool::run()
{
  QTE_D();

  auto cp = this->cameras();
  auto lp = this->landmarks();
  auto tp = this->tracks();

  // If cp is Null the initialize algorithm will create all cameras.
  // If not Null it will only create cameras if they are in the map but Null.
  // So we need to add placeholders for missing cameras to the map
  if (cp)
  {
    using kwiver::vital::frame_id_t;
    using kwiver::vital::camera_map;
    std::set<frame_id_t> frame_ids = tp->all_frame_ids();
    camera_map::map_camera_t all_cams = cp->cameras();

    for (auto const& id : frame_ids)
    {
      if (all_cams.find(id) == all_cams.end())
      {
        all_cams[id] = kwiver::vital::camera_sptr();
      }
    }
    cp = std::make_shared<kwiver::vital::simple_camera_map>(all_cams);
  }

  // If lp is Null the initialize algorithm will create all landmarks.
  // If not Null it will only create landmarks if they are in the map but Null.
  // So we need to add placeholders for missing landmarks to the map
  if (lp)
  {
    using kwiver::vital::track_id_t;
    using kwiver::vital::landmark_map;
    std::set<track_id_t> track_ids = tp->all_track_ids();
    landmark_map::map_landmark_t all_lms = lp->landmarks();

    for (auto const& id : track_ids)
    {
      if (all_lms.find(id) == all_lms.end())
      {
        all_lms[id] = kwiver::vital::landmark_sptr();
      }
    }
    lp = std::make_shared<kwiver::vital::simple_landmark_map>(all_lms);
  }

  d->algorithm->initialize(cp, lp, tp);

  this->updateCameras(cp);
  this->updateLandmarks(lp);
}

//-----------------------------------------------------------------------------
bool InitCamerasLandmarksTool::callback_handler(camera_map_sptr cameras,
                                                landmark_map_sptr landmarks)
{
  // make a copy of the tool data
  auto data = std::make_shared<ToolData>();
  data->copyCameras(cameras);
  data->copyLandmarks(landmarks);

  emit updated(data);
  return !this->isCanceled();
}
