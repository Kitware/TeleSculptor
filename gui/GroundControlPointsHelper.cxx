/*ckwg +29
 * Copyright 2018 by Kitware, Inc.
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

// MAPTK includes
#include "GroundControlPointsHelper.h"

#include "CameraView.h"
#include "GroundControlPointsWidget.h"
#include "MainWindow.h"
#include "WorldView.h"
#include "vtkMaptkCamera.h"

QTE_IMPLEMENT_D_FUNC(GroundControlPointsHelper)

//-----------------------------------------------------------------------------
class GroundControlPointsHelperPrivate
{
public:
  MainWindow* mainWindow = nullptr;
};

//-----------------------------------------------------------------------------
GroundControlPointsHelper::GroundControlPointsHelper(QObject* parent)
  : QObject(parent)
  , d_ptr(new GroundControlPointsHelperPrivate)
{
  QTE_D();

  d->mainWindow = qobject_cast<MainWindow*>(parent);
  Q_ASSERT(d->mainWindow);

  GroundControlPointsWidget* worldWidget =
    d->mainWindow->worldView()->groundControlPointsWidget();
  GroundControlPointsWidget* cameraWidget =
    d->mainWindow->cameraView()->groundControlPointsWidget();

  connect(worldWidget, &GroundControlPointsWidget::pointPlaced,
          this, &GroundControlPointsHelper::addCameraViewPoint);
  connect(cameraWidget, &GroundControlPointsWidget::pointPlaced,
          this, &GroundControlPointsHelper::addWorldViewPoint);

  connect(worldWidget, &GroundControlPointsWidget::pointMoved,
          this, &GroundControlPointsHelper::moveCameraViewPoint);
  connect(cameraWidget, &GroundControlPointsWidget::pointMoved,
          this, &GroundControlPointsHelper::moveWorldViewPoint);
  connect(worldWidget, &GroundControlPointsWidget::pointDeleted,
          cameraWidget, &GroundControlPointsWidget::deletePoint);
  connect(cameraWidget, &GroundControlPointsWidget::pointDeleted,
          worldWidget, &GroundControlPointsWidget::deletePoint);
  connect(cameraWidget, &GroundControlPointsWidget::activePointChanged,
          worldWidget, &GroundControlPointsWidget::setActivePoint);
  connect(worldWidget, &GroundControlPointsWidget::activePointChanged,
          cameraWidget, &GroundControlPointsWidget::setActivePoint);
}

//-----------------------------------------------------------------------------
GroundControlPointsHelper::~GroundControlPointsHelper()
{
}

//-----------------------------------------------------------------------------
void GroundControlPointsHelper::addCameraViewPoint()
{
  QTE_D();

  vtkMaptkCamera* camera = d->mainWindow->activeCamera();
  if (!camera)
  {
    return;
  }

  GroundControlPointsWidget* worldWidget =
    d->mainWindow->worldView()->groundControlPointsWidget();
  kwiver::vital::vector_3d p = worldWidget->activePoint();

  double cameraPt[2];
  camera->ProjectPoint(p, cameraPt);
  GroundControlPointsWidget* cameraWidget =
    d->mainWindow->cameraView()->groundControlPointsWidget();
  cameraWidget->addPoint(cameraPt[0], cameraPt[1], 0.0);
  cameraWidget->render();

  emit this->pointCountChanged(worldWidget->numberOfPoints());
}

//-----------------------------------------------------------------------------
void GroundControlPointsHelper::addWorldViewPoint()
{
  QTE_D();

  vtkMaptkCamera* camera = d->mainWindow->activeCamera();
  if (!camera)
  {
    return;
  }

  GroundControlPointsWidget* cameraWidget =
    d->mainWindow->cameraView()->groundControlPointsWidget();
  kwiver::vital::vector_3d cameraPt = cameraWidget->activePoint();
  kwiver::vital::vector_3d p = camera->UnprojectPoint(cameraPt.data());
  GroundControlPointsWidget* worldWidget =
    d->mainWindow->worldView()->groundControlPointsWidget();
  worldWidget->addPoint(p);
  this->moveWorldViewPoint();
  cameraWidget->render();
  worldWidget->render();

  emit this->pointCountChanged(worldWidget->numberOfPoints());
}

//-----------------------------------------------------------------------------
void GroundControlPointsHelper::moveCameraViewPoint()
{
  QTE_D();

  vtkMaptkCamera* camera = d->mainWindow->activeCamera();
  if (!camera)
  {
    return;
  }

  GroundControlPointsWidget* worldWidget =
    d->mainWindow->worldView()->groundControlPointsWidget();
  int handleId = worldWidget->activeHandle();

  kwiver::vital::vector_3d p = worldWidget->activePoint();

  double cameraPt[2];
  camera->ProjectPoint(p, cameraPt);
  GroundControlPointsWidget* cameraWidget =
    d->mainWindow->cameraView()->groundControlPointsWidget();
  cameraWidget->movePoint(handleId, cameraPt[0], cameraPt[1], 0.0);
  cameraWidget->render();
}

//-----------------------------------------------------------------------------
void GroundControlPointsHelper::moveWorldViewPoint()
{
  QTE_D();

  vtkMaptkCamera* camera = d->mainWindow->activeCamera();
  if (!camera)
  {
    return;
  }

  GroundControlPointsWidget* cameraWidget =
    d->mainWindow->cameraView()->groundControlPointsWidget();
  GroundControlPointsWidget* worldWidget =
    d->mainWindow->worldView()->groundControlPointsWidget();

  int handleId = cameraWidget->activeHandle();

  kwiver::vital::vector_3d cameraPt = cameraWidget->activePoint();
  kwiver::vital::vector_3d pt = worldWidget->point(handleId);
  double depth = d->mainWindow->activeCamera()->Depth(pt);

  kwiver::vital::vector_3d p = camera->UnprojectPoint(cameraPt.data(), depth);
  worldWidget->movePoint(handleId, p.x(), p.y(), p.z());
  worldWidget->render();
}

//-----------------------------------------------------------------------------
void GroundControlPointsHelper::updateCameraViewPoints()
{
  QTE_D();

  vtkMaptkCamera* camera = d->mainWindow->activeCamera();
  if (!camera)
  {
    return;
  }

  GroundControlPointsWidget* worldWidget =
    d->mainWindow->worldView()->groundControlPointsWidget();

  GroundControlPointsWidget* cameraWidget =
    d->mainWindow->cameraView()->groundControlPointsWidget();

  int numCameraPts = cameraWidget->numberOfPoints();
  int numWorldPts = worldWidget->numberOfPoints();
  while (numCameraPts > numWorldPts)
  {
    cameraWidget->deletePoint(--numCameraPts);
  }

  for (int i = 0; i < numWorldPts; ++i)
  {
    kwiver::vital::vector_3d worldPt = worldWidget->point(i);
    double cameraPt[2];
    camera->ProjectPoint(worldPt, cameraPt);
    if (i >= numCameraPts)
    {
      cameraWidget->addPoint(cameraPt[0], cameraPt[1], 0.0);
    }
    else
    {
      cameraWidget->movePoint(i, cameraPt[0], cameraPt[1], 0.0);
    }
  }
}

//-----------------------------------------------------------------------------
void GroundControlPointsHelper::setGroundControlPoints(
  kwiver::vital::landmark_map const& lm)
{
  QTE_D();

  // Clear all existing ground control points before adding new ones.
  GroundControlPointsWidget* cameraWidget =
    d->mainWindow->cameraView()->groundControlPointsWidget();
  cameraWidget->clearPoints();
  GroundControlPointsWidget* worldWidget =
    d->mainWindow->worldView()->groundControlPointsWidget();
  worldWidget->clearPoints();

  auto const& groundControlPoints = lm.landmarks();
  foreach (auto const& lm, groundControlPoints)
  {
    auto const& pos = lm.second->loc();

    worldWidget->addPoint(pos);
    this->addCameraViewPoint();
  }
}

//-----------------------------------------------------------------------------
kwiver::vital::landmark_map_sptr
GroundControlPointsHelper::groundControlPoints() const
{
  QTE_D();

  GroundControlPointsWidget* worldWidget =
    d->mainWindow->worldView()->groundControlPointsWidget();
  int numWorldPts = worldWidget->numberOfPoints();

  kwiver::vital::landmark_map::map_landmark_t groundControlPoints;
  for (int i = 0; i < numWorldPts; ++i)
  {
    kwiver::vital::vector_3d pt = worldWidget->point(i);
    groundControlPoints[i] = std::make_shared<kwiver::vital::landmark_d>(pt);
  }

  return std::make_shared<kwiver::vital::simple_landmark_map>(
    groundControlPoints);
}

//-----------------------------------------------------------------------------
void GroundControlPointsHelper::enableWidgets(bool enable)
{
  QTE_D();
  d->mainWindow->worldView()->groundControlPointsWidget()->enableWidget(
    enable);
  d->mainWindow->cameraView()->groundControlPointsWidget()->enableWidget(
    enable);
}
