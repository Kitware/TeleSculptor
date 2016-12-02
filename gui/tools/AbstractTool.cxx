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

#include "AbstractTool.h"

#include <QtCore/QThread>

#include <atomic>

//-----------------------------------------------------------------------------
class AbstractToolPrivate : public QThread
{
public:
  AbstractToolPrivate(AbstractTool* q) : q_ptr(q) {}

  virtual void run() QTE_OVERRIDE;

  kwiver::vital::track_set_sptr tracks;
  kwiver::vital::camera_map_sptr cameras;
  kwiver::vital::landmark_map_sptr landmarks;

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
kwiver::vital::track_set_sptr AbstractTool::tracks() const
{
  QTE_D();
  return d->tracks;
}

//-----------------------------------------------------------------------------
kwiver::vital::camera_map_sptr AbstractTool::cameras() const
{
  QTE_D();
  return d->cameras;
}

//-----------------------------------------------------------------------------
AbstractTool::landmark_map_sptr AbstractTool::landmarks() const
{
  QTE_D();
  return d->landmarks;
}

//-----------------------------------------------------------------------------
void AbstractTool::cancel()
{
  QTE_D();
  d->cancelRequested = true;
}

//-----------------------------------------------------------------------------
void AbstractTool::setTracks(track_set_sptr const& newTracks)
{
  if (newTracks)
  {
    auto copiedTracks = std::vector<kwiver::vital::track_sptr>{};
    foreach (auto const& ti, newTracks->tracks())
    {
      copiedTracks.push_back(std::make_shared<kwiver::vital::track>(*ti));
    }
    this->updateTracks(
      std::make_shared<kwiver::vital::simple_track_set>(copiedTracks));
  }
  else
  {
    this->updateTracks({});
  }
}

//-----------------------------------------------------------------------------
void AbstractTool::setCameras(camera_map_sptr const& newCameras)
{
  if (newCameras)
  {
    auto copiedCameras = kwiver::vital::camera_map::map_camera_t{};
    foreach (auto const& ci, newCameras->cameras())
    {
      copiedCameras.insert(std::make_pair(ci.first, ci.second->clone()));
    }
    this->updateCameras(
      std::make_shared<kwiver::vital::simple_camera_map>(copiedCameras));
  }
  else
  {
    this->updateCameras({});
  }
}

//-----------------------------------------------------------------------------
void AbstractTool::setLandmarks(landmark_map_sptr const& newLandmarks)
{
  if (newLandmarks)
  {
    auto copiedLandmarks = kwiver::vital::landmark_map::map_landmark_t{};
    foreach (auto const& ci, newLandmarks->landmarks())
    {
      copiedLandmarks.insert(std::make_pair(ci.first, ci.second->clone()));
    }
    this->updateLandmarks(
      std::make_shared<kwiver::vital::simple_landmark_map>(copiedLandmarks));
  }
  else
  {
    this->updateCameras({});
  }
}

//-----------------------------------------------------------------------------
bool AbstractTool::execute(QWidget* window)
{
  QTE_D();

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
  return d->tracks && d->tracks->size();
}

//-----------------------------------------------------------------------------
bool AbstractTool::hasCameras() const
{
  QTE_D();
  return d->cameras && d->cameras->size();
}

//-----------------------------------------------------------------------------
bool AbstractTool::hasLandmarks() const
{
  QTE_D();
  return d->landmarks && d->landmarks->size();
}

//-----------------------------------------------------------------------------
void AbstractTool::updateTracks(track_set_sptr const& newTracks)
{
  QTE_D();
  d->tracks = newTracks;
}

//-----------------------------------------------------------------------------
void AbstractTool::updateCameras(camera_map_sptr const& newCameras)
{
  QTE_D();
  d->cameras = newCameras;
}

//-----------------------------------------------------------------------------
void AbstractTool::updateLandmarks(landmark_map_sptr const& newLandmarks)
{
  QTE_D();
  d->landmarks = newLandmarks;
}
