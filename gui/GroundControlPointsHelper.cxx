/*ckwg +29
 * Copyright 2018-2019 by Kitware, Inc.
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
#include "vtkMaptkPointPicker.h"
#include "vtkMaptkPointPlacer.h"

// VTK includes
#include <vtkPlane.h>
#include <vtkRenderer.h>

// vtk includes
#include <vtkHandleWidget.h>

namespace kv = kwiver::vital;
using id_t = kv::ground_control_point_id_t;

QTE_IMPLEMENT_D_FUNC(GroundControlPointsHelper)

//-----------------------------------------------------------------------------
class GroundControlPointsHelperPrivate
{
public:
  MainWindow* mainWindow = nullptr;
  id_t curId = 0;
  kv::ground_control_point_map::map_ground_control_point_t groundControlPoints;
  std::map<vtkHandleWidget*, id_t> gcpHandleIdMap;

  id_t addPoint();
  id_t removePoint(vtkHandleWidget*);
};

//-----------------------------------------------------------------------------
id_t GroundControlPointsHelperPrivate::addPoint()
{
  GroundControlPointsWidget* worldWidget =
    this->mainWindow->worldView()->groundControlPointsWidget();
  auto const pt = worldWidget->activePoint();
  this->groundControlPoints[curId] =
    std::make_shared<kv::ground_control_point>(pt);
  auto const w = worldWidget->handleWidget(worldWidget->activeHandle());
  this->gcpHandleIdMap[w] = curId;
  return curId++;
}

//-----------------------------------------------------------------------------
id_t GroundControlPointsHelperPrivate::removePoint(vtkHandleWidget* handle)
{
  auto const id = this->gcpHandleIdMap[handle];
  this->gcpHandleIdMap.erase(handle);
  this->groundControlPoints.erase(id);
  return id;
}

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

  // Set a point placer on the world widget.
  // This has to be set before the widget is enabled.
  worldWidget->setPointPlacer(vtkNew<vtkMaptkPointPlacer>());

  // connections
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
  kv::vector_3d p = worldWidget->activePoint();

  double cameraPt[2];
  camera->ProjectPoint(p, cameraPt);
  GroundControlPointsWidget* cameraWidget =
    d->mainWindow->cameraView()->groundControlPointsWidget();
  cameraWidget->addPoint(cameraPt[0], cameraPt[1], 0.0);
  cameraWidget->render();

  emit this->pointAdded(d->addPoint());
  emit this->pointCountChanged(d->groundControlPoints.size());
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
  kv::vector_3d cameraPt = cameraWidget->activePoint();
  // Use an arbitarily value for depth to ensure that the landmarks would
  // be between the camera center and the back-projected point.
  kv::vector_3d p =
    camera->UnprojectPoint(cameraPt.data(), 100 * camera->GetDistance());

  // Pick a point along the active camera direction and use the depth of the
  // point to back-project the camera view ground control point.
  GroundControlPointsWidget* worldWidget =
    d->mainWindow->worldView()->groundControlPointsWidget();
  vtkNew<vtkMaptkPointPicker> pointPicker;
  double distance = 0;
  double gOrigin[3] = { 0, 0, 0 };
  double gNormal[3] = { 0, 0, 1 };
  if (pointPicker->Pick3DPoint(
        camera->GetPosition(), p.data(), worldWidget->renderer()))
  {
    p = kwiver::vital::vector_3d(pointPicker->GetPickPosition());
    p = camera->UnprojectPoint(cameraPt.data(), camera->Depth(p));
  }
  else if (vtkPlane::IntersectWithLine(camera->GetPosition(),
                                       p.data(),
                                       gNormal,
                                       gOrigin,
                                       distance,
                                       p.data()))
  {
    // Find the point where the ray intersects the ground plane and use that.
  }
  else
  {
    // If nothing was picked, ensure that the back-projection uses the depth of
    // camera origin point
    p = camera->UnprojectPoint(cameraPt.data());
  }
  worldWidget->addPoint(p);
  worldWidget->render();

  emit this->pointAdded(d->addPoint());
  emit this->pointCountChanged(d->groundControlPoints.size());
}

//-----------------------------------------------------------------------------
void GroundControlPointsHelper::removePoint(int handleId)
{
  QTE_D();
  GroundControlPointsWidget* worldWidget =
    d->mainWindow->worldView()->groundControlPointsWidget();
  emit this->pointRemoved(d->removePoint(worldWidget->handleWidget(handleId)));
  emit this->pointCountChanged(d->groundControlPoints.size());
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

  kv::vector_3d p = worldWidget->activePoint();

  double cameraPt[2];
  camera->ProjectPoint(p, cameraPt);
  GroundControlPointsWidget* cameraWidget =
    d->mainWindow->cameraView()->groundControlPointsWidget();
  cameraWidget->movePoint(handleId, cameraPt[0], cameraPt[1], 0.0);
  cameraWidget->render();
  auto const gcpId = d->gcpHandleIdMap[worldWidget->handleWidget(handleId)];
  emit this->pointChanged(gcpId);
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

  kv::vector_3d cameraPt = cameraWidget->activePoint();
  kv::vector_3d pt = worldWidget->point(handleId);
  double depth = d->mainWindow->activeCamera()->Depth(pt);

  kv::vector_3d p = camera->UnprojectPoint(cameraPt.data(), depth);
  worldWidget->movePoint(handleId, p.x(), p.y(), p.z());
  worldWidget->render();
  auto const gcpId = d->gcpHandleIdMap[worldWidget->handleWidget(handleId)];
  emit this->pointChanged(gcpId);
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
    kv::vector_3d worldPt = worldWidget->point(i);
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
  kv::ground_control_point_map const& gcpm)
{
  QTE_D();

  // Clear all existing ground control points before adding new ones.
  GroundControlPointsWidget* cameraWidget =
    d->mainWindow->cameraView()->groundControlPointsWidget();
  cameraWidget->clearPoints();
  GroundControlPointsWidget* worldWidget =
    d->mainWindow->worldView()->groundControlPointsWidget();
  worldWidget->clearPoints();

  auto const& groundControlPoints = gcpm.ground_control_points();
  foreach (auto const& gcp, groundControlPoints)
  {
    auto const& pos = gcp.second->loc();

    worldWidget->addPoint(pos);
    this->addCameraViewPoint();
  }
}

//-----------------------------------------------------------------------------
kv::ground_control_point_map_sptr
GroundControlPointsHelper::groundControlPoints() const
{
  QTE_D();
  return std::make_shared<kv::ground_control_point_map>(
    d->groundControlPoints);
}

//-----------------------------------------------------------------------------
bool GroundControlPointsHelper::readGroundControlPoints(QString const& path)
{
  // TODO
  return false;
}

//-----------------------------------------------------------------------------
bool GroundControlPointsHelper::writeGroundControlPoints(
  QString const& path, QWidget* dialogParent) const
{
  // TODO
  return false;
}

//-----------------------------------------------------------------------------
void GroundControlPointsHelper::enableWidgets(bool enable)
{
  QTE_D();
  d->mainWindow->worldView()->groundControlPointsWidget()->enableWidget(enable);
  d->mainWindow->cameraView()->groundControlPointsWidget()->enableWidget(
    enable);
}

//-----------------------------------------------------------------------------
kv::ground_control_point_sptr GroundControlPointsHelper::groundControlPoint(
  id_t pointId)
{
  QTE_D();
  return d->groundControlPoints[pointId];
}
