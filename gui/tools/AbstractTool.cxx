/*ckwg +29
 * Copyright 2017 by Kitware, Inc.
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

#include "AbstractTool.h"

#include <qtStlUtil.h>

#include <QThread>

#include <atomic>

//-----------------------------------------------------------------------------
class AbstractToolPrivate : public QThread
{
public:
  AbstractToolPrivate(AbstractTool* q)
    : data(std::make_shared<ToolData>()), q_ptr(q) {}

  virtual void run() override;

  std::shared_ptr<ToolData> data;

  std::atomic<bool> cancelRequested;

protected:
  QTE_DECLARE_PUBLIC_PTR(AbstractTool)
  QTE_DECLARE_PUBLIC(AbstractTool)
};

QTE_IMPLEMENT_D_FUNC(AbstractTool)

//-----------------------------------------------------------------------------
void AbstractToolPrivate::run()
{
  QTE_Q();
  q->run();
}

//-----------------------------------------------------------------------------
void ToolData::copyTracks(feature_track_set_sptr const& newTracks)
{
  if (newTracks)
  {
    this->tracks = std::dynamic_pointer_cast<kwiver::vital::feature_track_set>(
      newTracks->clone());
  }
  else
  {
    this->tracks = feature_track_set_sptr();
  }
}

//-----------------------------------------------------------------------------
void ToolData::copyCameras(camera_map_sptr const& newCameras)
{
  if (newCameras)
  {
    auto copiedCameras = kwiver::vital::camera_map::map_camera_t{};
    foreach (auto const& ci, newCameras->cameras())
    {
      copiedCameras.insert(std::make_pair(ci.first, ci.second->clone()));
    }
    this->cameras =
      std::make_shared<kwiver::vital::simple_camera_map>(copiedCameras);
  }
  else
  {
    this->cameras = camera_map_sptr();
  }
}

//-----------------------------------------------------------------------------
void ToolData::copyLandmarks(landmark_map_sptr const& newLandmarks)
{
  if (newLandmarks)
  {
    auto copiedLandmarks = kwiver::vital::landmark_map::map_landmark_t{};
    foreach (auto const& ci, newLandmarks->landmarks())
    {
      copiedLandmarks.insert(std::make_pair(ci.first, ci.second->clone()));
    }
    this->landmarks =
      std::make_shared<kwiver::vital::simple_landmark_map>(copiedLandmarks);
  }
  else
  {
    this->landmarks = landmark_map_sptr();
  }
}

//-----------------------------------------------------------------------------
void ToolData::copyDepth(depth_sptr const& newDepth)
{
  this->active_depth = vtkSmartPointer<vtkImageData>::New();
  if (newDepth)
  {
    this->active_depth->DeepCopy(newDepth);
  }
}

//-----------------------------------------------------------------------------
AbstractTool::AbstractTool(QObject* parent)
  : QAction(parent), d_ptr(new AbstractToolPrivate(this))
{
  QTE_D();
  connect(d, SIGNAL(finished()), this, SIGNAL(completed()));
}

//-----------------------------------------------------------------------------
AbstractTool::~AbstractTool()
{
  QTE_D();
  d->wait();
}

//-----------------------------------------------------------------------------
std::shared_ptr<ToolData> AbstractTool::data()
{
  QTE_D();
  return d->data;
}

//-----------------------------------------------------------------------------
unsigned int AbstractTool::activeFrame() const
{
  QTE_D();
  return d->data->activeFrame;
}

//-----------------------------------------------------------------------------
kwiver::vital::feature_track_set_sptr AbstractTool::tracks() const
{
  QTE_D();
  return d->data->tracks;
}

//-----------------------------------------------------------------------------
kwiver::vital::camera_map_sptr AbstractTool::cameras() const
{
  QTE_D();
  return d->data->cameras;
}

//-----------------------------------------------------------------------------
AbstractTool::landmark_map_sptr AbstractTool::landmarks() const
{
  QTE_D();
  return d->data->landmarks;
}

//-----------------------------------------------------------------------------
void AbstractTool::cancel()
{
  QTE_D();
  d->cancelRequested = true;
}

//-----------------------------------------------------------------------------
void AbstractTool::setActiveFrame(unsigned int frame)
{
  QTE_D();
  d->data->activeFrame = frame;
}

//-----------------------------------------------------------------------------
void AbstractTool::setTracks(feature_track_set_sptr const& newTracks)
{
  QTE_D();
  d->data->copyTracks(newTracks);
}

//-----------------------------------------------------------------------------
void AbstractTool::setCameras(camera_map_sptr const& newCameras)
{
  QTE_D();
  d->data->copyCameras(newCameras);
}

//-----------------------------------------------------------------------------
void AbstractTool::setLandmarks(landmark_map_sptr const& newLandmarks)
{
  QTE_D();
  d->data->copyLandmarks(newLandmarks);
}

//-----------------------------------------------------------------------------
void AbstractTool::setVideoPath(std::string const& path)
{
  QTE_D();
  d->data->videoPath = path;
}

//-----------------------------------------------------------------------------
void AbstractTool::setMaskPath(std::string const& path)
{
  QTE_D();
  d->data->maskPath = path;
}

//-----------------------------------------------------------------------------
void AbstractTool::setConfig(config_block_sptr& config)
{
  QTE_D();
  d->data->config = config;
}

//-----------------------------------------------------------------------------
bool AbstractTool::execute(QWidget* window)
{
  QTE_D();

  // Reset progress reporting parameters
  this->updateProgress(0);
  this->setDescription("");
  d->cancelRequested = false;
  d->start();
  return true;
}

//-----------------------------------------------------------------------------
bool AbstractTool::isCanceled() const
{
  QTE_D();
  return d->cancelRequested;
}

//-----------------------------------------------------------------------------
bool AbstractTool::hasTracks() const
{
  QTE_D();
  return d->data->tracks && d->data->tracks->size();
}

//-----------------------------------------------------------------------------
bool AbstractTool::hasCameras() const
{
  QTE_D();
  return d->data->cameras && d->data->cameras->size();
}

//-----------------------------------------------------------------------------
bool AbstractTool::hasLandmarks() const
{
  QTE_D();
  return d->data->landmarks && d->data->landmarks->size();
}

//-----------------------------------------------------------------------------
bool AbstractTool::hasVideoSource() const
{
  QTE_D();
  if (d->data->videoPath == "" || !d->data->config)
  {
    return false;
  }
  else
  {
    return true;
  }
}

//-----------------------------------------------------------------------------
void AbstractTool::updateTracks(feature_track_set_sptr const& newTracks)
{
  QTE_D();
  d->data->tracks = newTracks;
}

//-----------------------------------------------------------------------------
void AbstractTool::updateDepth(depth_sptr const& newDepth)
{
  QTE_D();
  d->data->active_depth = newDepth;
}

//-----------------------------------------------------------------------------
void AbstractTool::updateCameras(camera_map_sptr const& newCameras)
{
  QTE_D();
  d->data->cameras = newCameras;
}

//-----------------------------------------------------------------------------
void AbstractTool::updateLandmarks(landmark_map_sptr const& newLandmarks)
{
  QTE_D();
  d->data->landmarks = newLandmarks;
}

//-----------------------------------------------------------------------------
int AbstractTool::progress() const
{
  QTE_D();
  int p = d->data->progress;
  p = p < 0 ? 0 : p;
  p = p > 100 ? 100 : p;
  return p;
}

//-----------------------------------------------------------------------------
void AbstractTool::updateProgress(int value)
{
  QTE_D();
  d->data->progress = value;
}

//-----------------------------------------------------------------------------
QString AbstractTool::description() const
{
  QTE_D();
  return qtString(d->data->description);
}

//-----------------------------------------------------------------------------
void AbstractTool::setDescription(QString const& desc)
{
  QTE_D();
  d->data->description = stdString(desc);
}
