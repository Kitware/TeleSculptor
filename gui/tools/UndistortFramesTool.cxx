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

#include "UndistortFramesTool.h"

#include "maptk/undistort_frame.h"

#include <QtGui/QMessageBox>

//-----------------------------------------------------------------------------
class UndistortFramesToolPrivate
{
public:
  QMap<kwiver::vital::frame_id_t, QString> frames;
  QString dir;

};

QTE_IMPLEMENT_D_FUNC(UndistortFramesTool)

//-----------------------------------------------------------------------------
UndistortFramesTool::UndistortFramesTool(QObject* parent)
  : AbstractTool(parent), d_ptr(new UndistortFramesToolPrivate)
{
  this->setText("&Undistort Frames");
  this->setToolTip(
        "<nobr>Undistort frames.</nobr>");
}

//-----------------------------------------------------------------------------
UndistortFramesTool::~UndistortFramesTool()
{
}

//-----------------------------------------------------------------------------
AbstractTool::Outputs UndistortFramesTool::outputs() const
{
  return Cameras | Landmarks;
}

//-----------------------------------------------------------------------------
void UndistortFramesTool::setFrames(QMap<kwiver::vital::frame_id_t, QString> *
                                    const& newFrames)
{
  if (newFrames)
  {
    auto copiedFrames = QMap<kwiver::vital::frame_id_t, QString>();

    QMap<kwiver::vital::frame_id_t, QString>::const_iterator i =
        newFrames->constBegin();

    while (i != newFrames->constEnd()) {
        copiedFrames.insert(i.key(), i.value());
        ++i;
    }

    this->updateFrames(copiedFrames);
  }
  else
  {
    this->updateFrames(QMap<kwiver::vital::frame_id_t, QString>());
  }
}

void UndistortFramesTool::setOutputDir(const QString &dir)
{
  QTE_D();

  d->dir = dir;
}

//-----------------------------------------------------------------------------
void UndistortFramesTool::updateFrames(QMap<kwiver::vital::frame_id_t, QString>
                                       const& newFrames)
{
  QTE_D();
  d->frames = newFrames;
}

//-----------------------------------------------------------------------------
QMap<kwiver::vital::frame_id_t, QString> UndistortFramesTool::frames()
{
  QTE_D();

  return d->frames;
}

//-----------------------------------------------------------------------------
bool UndistortFramesTool::execute(QWidget* window)
{
  return AbstractTool::execute(window);
}

//-----------------------------------------------------------------------------
void UndistortFramesTool::run()
{
  QTE_D();

  auto cp = this->cameras();
  auto frames = this->frames();

  int nbCams = cp.get()->cameras().size();

  std::vector<std::string> frameList;
  std::vector<kwiver::vital::camera_sptr> cameras;

  for (int i = 0; i < nbCams; ++i)
  {
    frameList.push_back(frames[i].toStdString());
    cameras.push_back(cp.get()->cameras().at(i));
  }

  kwiver::maptk::undistortFrames(frameList,cameras,d->dir.toStdString());
}
