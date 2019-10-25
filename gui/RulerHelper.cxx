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

#include "RulerHelper.h"

#include "CameraView.h"
#include "MainWindow.h"
#include "RulerWidget.h"
#include "WorldView.h"
#include "vtkMaptkCamera.h"
#include "vtkMaptkPointPicker.h"
#include "vtkMaptkPointPlacer.h"

#include <vital/types/geodesy.h>

#include <vtkHandleWidget.h>
#include <vtkPlane.h>
#include <vtkRenderer.h>

namespace kv = kwiver::vital;

//-----------------------------------------------------------------------------
class RulerHelperPrivate
{
public:
  RulerHelperPrivate(RulerHelper* q)
    : q_ptr{ q }
  {
  }

  MainWindow* mainWindow = nullptr;

  void updateCameraViewRulerTickDistance();

private:
  QTE_DECLARE_PUBLIC_PTR(RulerHelper)
  QTE_DECLARE_PUBLIC(RulerHelper)
};

QTE_IMPLEMENT_D_FUNC(RulerHelper)

//-----------------------------------------------------------------------------
void RulerHelperPrivate::updateCameraViewRulerTickDistance()
{
  vtkMaptkCamera* camera = this->mainWindow->activeCamera();
  if (!camera)
  {
    return;
  }

  RulerWidget* worldWidget = this->mainWindow->worldView()->rulerWidget();
  RulerWidget* cameraWidget = this->mainWindow->cameraView()->rulerWidget();

  kv::vector_3d p1 = worldWidget->point1WorldPosition();
  kv::vector_3d p2 = worldWidget->point2WorldPosition();
  kv::vector_3d cp1 = cameraWidget->point1WorldPosition();
  kv::vector_3d cp2 = cameraWidget->point2WorldPosition();
  double p21[3];
  p21[0] = p2[0] - p1[0];
  p21[1] = p2[1] - p1[1];
  p21[2] = p2[2] - p1[2];
  double dist = vtkMath::Norm(p21);
  double numTicks = dist / worldWidget->rulerTickDistance();
  p21[0] = cp2[0] - cp1[0];
  p21[1] = cp2[1] - cp1[1];
  p21[2] = 0;
  dist = vtkMath::Norm(p21);
  cameraWidget->setRulerTickDistance(dist / numTicks);
}

//-----------------------------------------------------------------------------
RulerHelper::RulerHelper(QObject* parent)
  : QObject{ parent }
  , d_ptr{ new RulerHelperPrivate{ this } }
{
  QTE_D();

  d->mainWindow = qobject_cast<MainWindow*>(parent);
  Q_ASSERT(d->mainWindow);

  RulerWidget* worldWidget = d->mainWindow->worldView()->rulerWidget();
  RulerWidget* cameraWidget = d->mainWindow->cameraView()->rulerWidget();

  QObject::connect(worldWidget,
                   &RulerWidget::pointPlaced,
                   this,
                   &RulerHelper::addCameraViewPoint);
  QObject::connect(cameraWidget,
                   &RulerWidget::pointPlaced,
                   this,
                   &RulerHelper::addWorldViewPoint);
  QObject::connect(worldWidget,
                   &RulerWidget::pointMoved,
                   this,
                   &RulerHelper::moveCameraViewPoint);
  QObject::connect(cameraWidget,
                   &RulerWidget::pointMoved,
                   this,
                   &RulerHelper::moveWorldViewPoint);

  // Set a point placer on the world widget.
  // This has to be set before the widget is enabled.
  worldWidget->setPointPlacer(vtkNew<vtkMaptkPointPlacer>());
}

//-----------------------------------------------------------------------------
RulerHelper::~RulerHelper()
{
}

//-----------------------------------------------------------------------------
void RulerHelper::addWorldViewPoint(int pointId)
{
  QTE_D();

  vtkMaptkCamera* camera = d->mainWindow->activeCamera();
  if (!camera)
  {
    return;
  }

  RulerWidget* worldWidget = d->mainWindow->worldView()->rulerWidget();
  RulerWidget* cameraWidget = d->mainWindow->cameraView()->rulerWidget();

  kv::vector_3d cameraPt = (pointId == 0 ? cameraWidget->point1WorldPosition()
                                         : cameraWidget->point2WorldPosition());

  // Use an arbitarily value for depth to ensure that the landmarks would
  // be between the camera center and the back-projected point.
  kv::vector_3d p =
    camera->UnprojectPoint(cameraPt.data(), 100 * camera->GetDistance());

  // Pick a point along the active camera direction and use the depth of the
  // point to back-project the camera view ground control point.
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

  if (pointId == 0)
  {
    worldWidget->setPoint1WorldPosition(p[0], p[1], p[2]);
  }
  else
  {
    worldWidget->setPoint2WorldPosition(p[0], p[1], p[2]);
    worldWidget->render();
    cameraWidget->setDistance(worldWidget->distance());
    d->updateCameraViewRulerTickDistance();
    cameraWidget->render();
  }
}

//-----------------------------------------------------------------------------
void RulerHelper::moveCameraViewPoint(int pointId)
{
  QTE_D();

  vtkMaptkCamera* camera = d->mainWindow->activeCamera();
  if (!camera)
  {
    return;
  }

  RulerWidget* worldWidget = d->mainWindow->worldView()->rulerWidget();
  RulerWidget* cameraWidget = d->mainWindow->cameraView()->rulerWidget();

  kv::vector_3d worldPt = (pointId == 0 ? worldWidget->point1WorldPosition()
                                        : worldWidget->point2WorldPosition());

  double cameraPt[2];
  camera->ProjectPoint(worldPt, cameraPt);
  if (pointId == 0)
  {
    cameraWidget->setPoint1WorldPosition(cameraPt[0], cameraPt[1], 0.0);
  }
  else
  {
    cameraWidget->setPoint2WorldPosition(cameraPt[0], cameraPt[1], 0.0);
  }
  cameraWidget->setDistance(worldWidget->distance());
  d->updateCameraViewRulerTickDistance();
  cameraWidget->render();
}

//-----------------------------------------------------------------------------
void RulerHelper::moveWorldViewPoint(int pointId)
{
  QTE_D();

  vtkMaptkCamera* camera = d->mainWindow->activeCamera();
  if (!camera)
  {
    return;
  }

  RulerWidget* worldWidget = d->mainWindow->worldView()->rulerWidget();
  RulerWidget* cameraWidget = d->mainWindow->cameraView()->rulerWidget();

  kv::vector_3d cameraPt = (pointId == 0 ? cameraWidget->point1WorldPosition()
                                         : cameraWidget->point2WorldPosition());

  kv::vector_3d worldPt = (pointId == 0 ? worldWidget->point1WorldPosition()
                                        : worldWidget->point2WorldPosition());

  double depth = camera->Depth(worldPt);
  kv::vector_3d p = camera->UnprojectPoint(cameraPt.data(), depth);
  if (pointId == 0)
  {
    worldWidget->setPoint1WorldPosition(p[0], p[1], p[2]);
  }
  else
  {
    worldWidget->setPoint2WorldPosition(p[0], p[1], p[2]);
  }
  worldWidget->render();
  cameraWidget->setDistance(worldWidget->distance());
  d->updateCameraViewRulerTickDistance();
  cameraWidget->render();
}

//-----------------------------------------------------------------------------
void RulerHelper::addCameraViewPoint(int pointId)
{
  QTE_D();
  vtkMaptkCamera* camera = d->mainWindow->activeCamera();
  if (!camera)
  {
    return;
  }

  RulerWidget* worldWidget = d->mainWindow->worldView()->rulerWidget();
  RulerWidget* cameraWidget = d->mainWindow->cameraView()->rulerWidget();

  kv::vector_3d p = worldWidget->point1WorldPosition();
  double cameraPt[2];
  camera->ProjectPoint(p, cameraPt);
  cameraWidget->setPoint1WorldPosition(cameraPt[0], cameraPt[1], 0.0);
  cameraWidget->setDistance(worldWidget->distance());

  if (pointId == 1)
  {
    kv::vector_3d p2 = worldWidget->point2WorldPosition();
    double cameraPt2[2];
    camera->ProjectPoint(p2, cameraPt2);
    cameraWidget->setPoint2WorldPosition(cameraPt2[0], cameraPt2[1], 0.0);
    d->updateCameraViewRulerTickDistance();
    cameraWidget->render();
  }
}

//-----------------------------------------------------------------------------
void RulerHelper::updateCameraViewRuler()
{
  QTE_D();
  vtkMaptkCamera* camera = d->mainWindow->activeCamera();
  if (!camera)
  {
    return;
  }

  RulerWidget* worldWidget = d->mainWindow->worldView()->rulerWidget();
  if (worldWidget->isRulerPlaced())
  {
    this->addCameraViewPoint(1);
  }
}

//-----------------------------------------------------------------------------
void RulerHelper::enableWidgets(bool enable)
{
  QTE_D();
  d->mainWindow->worldView()->rulerWidget()->enableWidget(enable);
  d->mainWindow->cameraView()->rulerWidget()->enableWidget(enable);
}

//-----------------------------------------------------------------------------
void RulerHelper::resetRuler()
{
  QTE_D();
  d->mainWindow->worldView()->rulerWidget()->removeRuler();
  d->mainWindow->cameraView()->rulerWidget()->removeRuler();
}
