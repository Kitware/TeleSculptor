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

#include "BundleAdjustTool.h"
#include "GuiCommon.h"

#include <vital/algo/bundle_adjust.h>

#include <QMessageBox>

using kwiver::vital::algo::bundle_adjust;
using kwiver::vital::algo::bundle_adjust_sptr;

namespace
{
static char const* const BLOCK = "bundle_adjuster";
}

//-----------------------------------------------------------------------------
class BundleAdjustToolPrivate
{
public:
  bundle_adjust_sptr algorithm;
};

QTE_IMPLEMENT_D_FUNC(BundleAdjustTool)

//-----------------------------------------------------------------------------
BundleAdjustTool::BundleAdjustTool(QObject* parent)
  : AbstractTool(parent), d_ptr(new BundleAdjustToolPrivate)
{
  this->setText("&Refine Solution");
  this->setToolTip(
    "<nobr>Apply bundle adjustment to the cameras and landmarks in order to"
    "</nobr> refine the quality of the 3D reconstruction");
}

//-----------------------------------------------------------------------------
BundleAdjustTool::~BundleAdjustTool()
{
}

//-----------------------------------------------------------------------------
AbstractTool::Outputs BundleAdjustTool::outputs() const
{
  return Cameras | Landmarks;
}

//-----------------------------------------------------------------------------
bool BundleAdjustTool::execute(QWidget* window)
{
  QTE_D();

  // Check inputs
  if (!this->hasLandmarks() || !this->hasCameras() || !this->hasTracks())
  {
    QMessageBox::information(
      window, "Insufficient data",
      "This operation requires feature tracks, cameras and landmarks.");
    return false;
  }

  // Merge project config with default config file
  auto const config = readConfig("gui_bundle_adjust.conf");

  // Check configuration
  if (!config)
  {
    QMessageBox::critical(
      window, "Configuration error",
      "No configuration data was found. Please check your installation.");
    return false;
  }

  config->merge_config(this->data()->config);
  if (!bundle_adjust::check_nested_algo_configuration(BLOCK, config))
  {
    QMessageBox::critical(
      window, "Configuration error",
      "An error was found in the algorithm configuration.");
    return false;
  }

  // Create algorithm from configuration
  bundle_adjust::set_nested_algo_configuration(BLOCK, config, d->algorithm);

  // Set the callback to receive updates
  using std::placeholders::_1;
  using std::placeholders::_2;
  typedef bundle_adjust::callback_t callback_t;
  callback_t cb = std::bind(&BundleAdjustTool::callback_handler, this, _1, _2);
  d->algorithm->set_callback(cb);

  // Hand off to base class
  return AbstractTool::execute(window);
}

//-----------------------------------------------------------------------------
void BundleAdjustTool::run()
{
  QTE_D();

  auto cp = this->cameras();
  auto lp = this->landmarks();
  auto tp = this->tracks();

  d->algorithm->optimize(cp, lp, tp);

  this->updateCameras(cp);
  this->updateLandmarks(lp);
}

//-----------------------------------------------------------------------------
bool BundleAdjustTool::callback_handler(camera_map_sptr cameras,
                                        landmark_map_sptr landmarks)
{
  // make a copy of the tool data
  auto data = std::make_shared<ToolData>();
  data->copyCameras(cameras);
  data->copyLandmarks(landmarks);

  emit updated(data);
  return !this->isCanceled();
}
