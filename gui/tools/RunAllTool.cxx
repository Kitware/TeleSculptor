/*ckwg +29
 * Copyright 2019 by Kitware, Inc.
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

#include "RunAllTool.h"
#include "GuiCommon.h"

#include <arrows/core/depth_utils.h>

#include "BundleAdjustTool.h"
#include "ComputeAllDepthTool.h"
#include "FuseDepthTool.h"
#include "TrackFeaturesTool.h"
#include "InitCamerasLandmarksTool.h"

#include <QMessageBox>
#include <qtStlUtil.h>

#include <algorithm>

//-----------------------------------------------------------------------------
class RunAllToolPrivate
{
public:
  RunAllToolPrivate() : output(0) {}


  AbstractTool::Outputs output;

  QWidget* window;
};

QTE_IMPLEMENT_D_FUNC(RunAllTool)

//-----------------------------------------------------------------------------
RunAllTool::RunAllTool(QObject* parent)
  : AbstractTool(parent), d_ptr(new RunAllToolPrivate)
{
  this->data()->logger =
    kwiver::vital::get_logger("telesculptor.tools.run_all");

  this->setText("&Runs all main tools");
  this->setToolTip("Computes 3D model from video");
}

//-----------------------------------------------------------------------------
RunAllTool::~RunAllTool()
{
}

//-----------------------------------------------------------------------------
AbstractTool::Outputs RunAllTool::outputs() const
{
  QTE_D();
  return d->output;
}

//-----------------------------------------------------------------------------
void RunAllTool::connectTool(AbstractTool* tool)
{
  QObject::connect(tool, &AbstractTool::updated,
                   this, &RunAllTool::forwardInterimResults);
  QObject::connect(tool, &AbstractTool::completed,
                   this, &RunAllTool::forwardFinalResults);
  QObject::connect(tool, &AbstractTool::failed,
                   this, &RunAllTool::reportToolError);
}

//-----------------------------------------------------------------------------
bool RunAllTool::execute(QWidget* window)
{
  QTE_D();

  if (!this->hasVideoSource())
  {
    QMessageBox::information(
      window, "Insufficient data", "This operation requires a video source.");
    return false;
  }

  d->window = window;

  return AbstractTool::execute(window);
}
//-----------------------------------------------------------------------------
void RunAllTool::forwardInterimResults(std::shared_ptr<ToolData> data)
{
  emit AbstractTool::updated(data);
}

//-----------------------------------------------------------------------------
void RunAllTool::forwardFinalResults()
{
}

//-----------------------------------------------------------------------------
void RunAllTool::reportToolError(QString const& msg)
{
  emit AbstractTool::failed(msg);
}

//-----------------------------------------------------------------------------
void RunAllTool::run()
{
  QTE_D();

  std::unique_ptr<TrackFeaturesTool> tracker = std::make_unique<TrackFeaturesTool>();
  connectTool(tracker.get());
  tracker->setToolData(this->data());
  d->output = Tracks;
  tracker->execute(d->window);
  tracker->wait();
  
  emit AbstractTool::saved();
  tracker->disconnect();
  tracker.reset(); //delete tracker

  std::unique_ptr<InitCamerasLandmarksTool> initializer = std::make_unique<InitCamerasLandmarksTool>();
  connectTool(initializer.get());
  initializer->setToolData(this->data());
  d->output = Cameras | Landmarks | TrackChanges | Tracks;
  initializer->execute(d->window);
  initializer->wait();
  
  emit AbstractTool::saved();
  initializer->disconnect();
  initializer.reset();

  std::unique_ptr<BundleAdjustTool> bundler = std::make_unique<BundleAdjustTool>();
  connectTool(bundler.get());
  bundler->setToolData(this->data());
  d->output = Cameras | Landmarks;
  bundler->execute(d->window);
  bundler->wait();  
  emit AbstractTool::saved();
  bundler->disconnect();
  bundler.reset();

  //Compute an ROI from landmarks
  auto const& lm = this->landmarks()->landmarks();
  std::vector<kwiver::vital::landmark_sptr> landmarks_out;
  foreach (auto const& l, lm)
  {
    landmarks_out.push_back(l.second);
  }
  double bounds[6];
  kwiver::arrows::core::compute_robust_ROI(landmarks_out, bounds);
  vtkNew<vtkBox> roi;
  roi->AddBounds(bounds);
  setROI(roi);

  std::unique_ptr<ComputeAllDepthTool> depther = std::make_unique<ComputeAllDepthTool>();
  connectTool(depther.get());
  depther->setToolData(this->data());
  d->output = Depth | ActiveFrame | BatchDepth;
  depther->execute(d->window);
  depther->wait();
  emit AbstractTool::saved();
  depther->disconnect();
  depther.reset();

  std::unique_ptr<FuseDepthTool> fuser = std::make_unique<FuseDepthTool>();
  connectTool(fuser.get());
  fuser->setToolData(this->data());
  d->output = Fusion;
  fuser->execute(d->window);
  fuser->wait();
  emit AbstractTool::saved();
  fuser->disconnect();
  fuser.reset();

  d->output = 0;
}
