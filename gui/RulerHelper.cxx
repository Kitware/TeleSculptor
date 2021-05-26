// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#include "RulerHelper.h"

#include "CameraView.h"
#include "MainWindow.h"
#include "RulerWidget.h"
#include "WorldView.h"
#include "vtkMaptkPointPicker.h"
#include "vtkMaptkPointPlacer.h"

#include "arrows/vtk/vtkKwiverCamera.h"
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
  QTE_Q();
  kwiver::arrows::vtk::vtkKwiverCamera* camera = this->mainWindow->activeCamera();
  if (!camera)
  {
    return;
  }

  RulerWidget* worldWidget = q->worldWidget();
  RulerWidget* cameraWidget = q->cameraWidget();

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

  RulerWidget* worldWidget = this->worldWidget();
  RulerWidget* cameraWidget = this->cameraWidget();

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

  kwiver::arrows::vtk::vtkKwiverCamera* camera = d->mainWindow->activeCamera();
  if (!camera)
  {
    return;
  }

  RulerWidget* worldWidget = this->worldWidget();
  RulerWidget* cameraWidget = this->cameraWidget();

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

  kwiver::arrows::vtk::vtkKwiverCamera* camera = d->mainWindow->activeCamera();
  if (!camera)
  {
    return;
  }

  RulerWidget* worldWidget = this->worldWidget();
  RulerWidget* cameraWidget = this->cameraWidget();

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

  kwiver::arrows::vtk::vtkKwiverCamera* camera = d->mainWindow->activeCamera();
  if (!camera)
  {
    return;
  }

  RulerWidget* worldWidget = this->worldWidget();
  RulerWidget* cameraWidget = this->cameraWidget();

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
  kwiver::arrows::vtk::vtkKwiverCamera* camera = d->mainWindow->activeCamera();
  if (!camera)
  {
    return;
  }

  RulerWidget* worldWidget = this->worldWidget();
  RulerWidget* cameraWidget = this->cameraWidget();

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
  kwiver::arrows::vtk::vtkKwiverCamera* camera = d->mainWindow->activeCamera();
  if (!camera)
  {
    return;
  }

  RulerWidget* worldWidget = this->worldWidget();
  if (worldWidget->isRulerPlaced())
  {
    this->addCameraViewPoint(1);
  }
}

//-----------------------------------------------------------------------------
void RulerHelper::enableWidgets(bool enable)
{
  this->worldWidget()->enableWidget(enable);
  this->cameraWidget()->enableWidget(enable);
}

//-----------------------------------------------------------------------------
void RulerHelper::resetRuler()
{
  this->worldWidget()->removeRuler();
  this->cameraWidget()->removeRuler();
}

//-----------------------------------------------------------------------------
RulerWidget* RulerHelper::worldWidget()
{
  QTE_D();
  return d->mainWindow->worldView()->rulerWidget();
}

//-----------------------------------------------------------------------------
RulerWidget* RulerHelper::cameraWidget()
{
  QTE_D();
  return d->mainWindow->cameraView()->rulerWidget();
}

//-----------------------------------------------------------------------------
void RulerHelper::setRulerTickDistance(double dist)
{
  QTE_D();
  this->worldWidget()->setRulerTickDistance(dist);
  d->updateCameraViewRulerTickDistance();
}

//-----------------------------------------------------------------------------
void RulerHelper::setRulerColor(const QColor& rgb)
{
  this->worldWidget()->setRulerColor(rgb);
  this->cameraWidget()->setRulerColor(rgb);
}
