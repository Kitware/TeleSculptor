/*ckwg +29
 * Copyright 2015 by Kitware, Inc.
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
 *  * Neither name of Kitware, Inc. nor the names of any contributors may be used
 *    to endorse or promote products derived from this software without specific
 *    prior written permission.
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

#include "WorldView.h"

#include "ui_WorldView.h"
#include "am_WorldView.h"

#include "CameraOptions.h"
#include "PointOptions.h"
#include "vtkMaptkCamera.h"
#include "vtkMaptkCameraRepresentation.h"

#include <maptk/camera.h>
#include <maptk/landmark_map.h>

#include <vtkActorCollection.h>
#include <vtkBoundingBox.h>
#include <vtkCellArray.h>
#include <vtkNew.h>
#include <vtkPlaneSource.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>

#include <QtGui/QMenu>
#include <QtGui/QToolButton>
#include <QtGui/QWidgetAction>

//-----------------------------------------------------------------------------
class WorldViewPrivate
{
public:
  WorldViewPrivate() : cameraRepDirty(false), scaleDirty(false) {}

  void setPopup(QAction* action, QMenu* menu);
  void setPopup(QAction* action, QWidget* widget);

  void setView(maptk::vector_3d const& normal, maptk::vector_3d const& up);
  void setView(double ni, double nj, double nk,
               double ui, double uj, double uk);

  void updateCameras(WorldView*);
  void updateScale(WorldView*);

  Ui::WorldView UI;
  Am::WorldView AM;

  vtkNew<vtkRenderer> renderer;
  vtkNew<vtkRenderWindow> renderWindow;

  vtkNew<vtkMaptkCameraRepresentation> cameraRep;

  QList<vtkActor*> landmarkActors;

  vtkNew<vtkPlaneSource> groundPlane;
  vtkNew<vtkActor> groundActor;

  CameraOptions* cameraOptions;
  PointOptions* landmarkOptions;

  bool cameraRepDirty;
  bool scaleDirty;
};

QTE_IMPLEMENT_D_FUNC(WorldView)

//-----------------------------------------------------------------------------
void WorldViewPrivate::setPopup(QAction* action, QMenu* menu)
{
  auto const widget = this->UI.toolBar->widgetForAction(action);
  auto const button = qobject_cast<QToolButton*>(widget);

  if (button)
  {
    button->setPopupMode(QToolButton::MenuButtonPopup);
    button->setMenu(menu);
  }
}

//-----------------------------------------------------------------------------
void WorldViewPrivate::setPopup(QAction* action, QWidget* widget)
{
  auto const parent = action->parentWidget();

  auto const proxy = new QWidgetAction(parent);
  proxy->setDefaultWidget(widget);

  auto const menu = new QMenu(parent);
  menu->addAction(proxy);

  this->setPopup(action, menu);
}

//-----------------------------------------------------------------------------
void WorldViewPrivate::setView(
  double ni, double nj, double nk, double ui, double uj, double uk)
{
  this->setView(maptk::vector_3d(ni, nj, nk), maptk::vector_3d(ui, uj, uk));
}

//-----------------------------------------------------------------------------
void WorldViewPrivate::setView(
  maptk::vector_3d const& normal, maptk::vector_3d const& up)
{
  // Get camera
  auto const camera = this->renderer->GetActiveCamera();

  // Get camera parameters
  auto focus = maptk::vector_3d();
  auto pos = maptk::vector_3d();
  camera->GetPosition(pos.data());
  camera->GetFocalPoint(focus.data());

  // Compute new position
  pos = focus + (normal * (focus - pos).norm());

  // Modify camera
  camera->SetPosition(pos.data());
  camera->SetViewUp(up.data());

  this->UI.renderWidget->update();
}

//-----------------------------------------------------------------------------
void WorldViewPrivate::updateCameras(WorldView* q)
{
  if (!this->cameraRepDirty)
  {
    this->cameraRepDirty = true;
    QMetaObject::invokeMethod(q, "updateCameras", Qt::QueuedConnection);

    // If the cameras are dirty, then the camera scale must also be dirty
    this->updateScale(q);
  }
}

//-----------------------------------------------------------------------------
void WorldViewPrivate::updateScale(WorldView* q)
{
  if (!this->scaleDirty)
  {
    this->scaleDirty = true;
    QMetaObject::invokeMethod(q, "updateScale", Qt::QueuedConnection);
  }
}

//-----------------------------------------------------------------------------
WorldView::WorldView(QWidget* parent, Qt::WindowFlags flags)
  : QWidget(parent, flags), d_ptr(new WorldViewPrivate)
{
  QTE_D();

  // Set up UI
  d->UI.setupUi(this);
  d->AM.setupActions(d->UI, this);

  auto const viewMenu = new QMenu(this);
  viewMenu->addAction(d->UI.actionViewReset);
  viewMenu->addAction(d->UI.actionViewResetLandmarks);
  viewMenu->addSeparator();
  viewMenu->addAction(d->UI.actionViewToWorldTop);
  viewMenu->addAction(d->UI.actionViewToWorldLeft);
  viewMenu->addAction(d->UI.actionViewToWorldRight);
  viewMenu->addAction(d->UI.actionViewToWorldFront);
  viewMenu->addAction(d->UI.actionViewToWorldBack);
  viewMenu->addSeparator();
  viewMenu->addAction(d->UI.actionViewPerspective);
  d->setPopup(d->UI.actionViewReset, viewMenu);

  d->cameraOptions = new CameraOptions(d->cameraRep.GetPointer(), this);
  d->setPopup(d->UI.actionShowCameras, d->cameraOptions);

  connect(d->cameraOptions, SIGNAL(modified()),
          d->UI.renderWidget, SLOT(update()));

  d->landmarkOptions = new PointOptions("WorldView/Landmarks", this);
  d->setPopup(d->UI.actionShowLandmarks, d->landmarkOptions);

  connect(d->landmarkOptions, SIGNAL(modified()),
          d->UI.renderWidget, SLOT(update()));

  // Connect actions
  this->addAction(d->UI.actionViewReset);
  this->addAction(d->UI.actionViewResetLandmarks);
  this->addAction(d->UI.actionViewPerspective);
  this->addAction(d->UI.actionShowCameras);
  this->addAction(d->UI.actionShowLandmarks);
  this->addAction(d->UI.actionShowGroundPlane);

  connect(d->UI.actionViewReset, SIGNAL(triggered()),
          this, SLOT(resetView()));
  connect(d->UI.actionViewResetLandmarks, SIGNAL(triggered()),
          this, SLOT(resetViewToLandmarks()));
  connect(d->UI.actionViewToWorldTop, SIGNAL(triggered()),
          this, SLOT(viewToWorldTop()));
  connect(d->UI.actionViewToWorldLeft, SIGNAL(triggered()),
          this, SLOT(viewToWorldLeft()));
  connect(d->UI.actionViewToWorldRight, SIGNAL(triggered()),
          this, SLOT(viewToWorldRight()));
  connect(d->UI.actionViewToWorldFront, SIGNAL(triggered()),
          this, SLOT(viewToWorldFront()));
  connect(d->UI.actionViewToWorldBack, SIGNAL(triggered()),
          this, SLOT(viewToWorldBack()));
  connect(d->UI.actionViewPerspective, SIGNAL(toggled(bool)),
          this, SLOT(setPerspective(bool)));

  connect(d->UI.actionShowCameras, SIGNAL(toggled(bool)),
          this, SLOT(setCamerasVisible(bool)));
  connect(d->UI.actionShowLandmarks, SIGNAL(toggled(bool)),
          this, SLOT(setLandmarksVisible(bool)));
  connect(d->UI.actionShowGroundPlane, SIGNAL(toggled(bool)),
          this, SLOT(setGroundPlaneVisible(bool)));

  // Set up render pipeline
  d->renderer->SetBackground(0, 0, 0);
  d->renderWindow->AddRenderer(d->renderer.GetPointer());
  d->UI.renderWidget->SetRenderWindow(d->renderWindow.GetPointer());

  d->renderer->AddActor(d->cameraRep->GetNonActiveActor());
  d->renderer->AddActor(d->cameraRep->GetActiveActor());
  d->renderer->AddActor(d->cameraRep->GetPathActor());

  // Set up ground plane grid
  d->groundPlane->SetOrigin(-10.0, -10.0, 0.0);
  d->groundPlane->SetPoint1(+10.0, -10.0, 0.0);
  d->groundPlane->SetPoint2(-10.0, +10.0, 0.0);
  d->groundPlane->SetResolution(20, 20);

  vtkNew<vtkPolyDataMapper> mapper;
  mapper->SetInputConnection(d->groundPlane->GetOutputPort());

  d->groundActor->SetMapper(mapper.GetPointer());
  d->groundActor->GetProperty()->SetColor(0.5, 0.5, 0.5);
  d->groundActor->GetProperty()->SetLighting(false);
  d->groundActor->GetProperty()->SetRepresentationToWireframe();
  d->renderer->AddActor(d->groundActor.GetPointer());
}

//-----------------------------------------------------------------------------
WorldView::~WorldView()
{
}

//-----------------------------------------------------------------------------
void WorldView::addCamera(int id, vtkMaptkCamera* camera)
{
  Q_UNUSED(id)

  QTE_D();

  d->cameraRep->AddCamera(camera);

  d->updateCameras(this);
}

//-----------------------------------------------------------------------------
void WorldView::setActiveCamera(vtkMaptkCamera* camera)
{
  QTE_D();

  d->cameraRep->SetActiveCamera(camera);

  d->updateCameras(this);
}

//-----------------------------------------------------------------------------
void WorldView::addLandmarks(maptk::landmark_map const& lm)
{
  QTE_D();

  auto const& landmarks = lm.landmarks();

  vtkNew<vtkPoints> points;
  vtkNew<vtkCellArray> verts;

  points->Allocate(static_cast<vtkIdType>(landmarks.size()));
  verts->Allocate(static_cast<vtkIdType>(landmarks.size()));

  vtkIdType vertIndex = 0;
  foreach_iter (auto, lmi, landmarks)
  {
    auto const& pos = lmi->second->loc();
    points->InsertNextPoint(pos.data());
    verts->InsertNextCell(1);
    verts->InsertCellPoint(vertIndex++);
  }

  vtkNew<vtkPolyData> polyData;
  vtkNew<vtkPolyDataMapper> mapper;

  polyData->SetPoints(points.GetPointer());
  polyData->SetVerts(verts.GetPointer());
  mapper->SetInputData(polyData.GetPointer());

  vtkNew<vtkActor> actor;
  actor->SetMapper(mapper.GetPointer());

  d->renderer->AddActor(actor.GetPointer());
  d->landmarkOptions->addActor(actor.GetPointer());

  d->landmarkActors.append(actor.GetPointer());

  d->updateScale(this);
}

//-----------------------------------------------------------------------------
void WorldView::setCamerasVisible(bool state)
{
  QTE_D();
  d->cameraOptions->setCamerasVisible(state);
}

//-----------------------------------------------------------------------------
void WorldView::setLandmarksVisible(bool state)
{
  QTE_D();

  foreach (auto const actor, d->landmarkActors)
  {
    actor->SetVisibility(state);
  }

  d->UI.renderWidget->update();
}

//-----------------------------------------------------------------------------
void WorldView::setGroundPlaneVisible(bool state)
{
  QTE_D();

  d->groundActor->SetVisibility(state);
  d->UI.renderWidget->update();
}

//-----------------------------------------------------------------------------
void WorldView::resetView()
{
  QTE_D();

  d->renderer->ResetCamera();
  d->UI.renderWidget->update();
}

//-----------------------------------------------------------------------------
void WorldView::resetViewToLandmarks()
{
  QTE_D();

  vtkBoundingBox bbox;

  foreach (auto const actor, d->landmarkActors)
  {
    bbox.AddBounds(actor->GetBounds());
  }

  double bounds[6];
  bbox.GetBounds(bounds);
  d->renderer->ResetCamera(bounds);

  d->UI.renderWidget->update();
}

//-----------------------------------------------------------------------------
void WorldView::viewToWorldTop()
{
  QTE_D();
  d->setView(0.0, 0.0, +1.0, 0.0, +1.0, 0.0);
}

//-----------------------------------------------------------------------------
void WorldView::viewToWorldLeft()
{
  QTE_D();
  d->setView(-1.0, 0.0, 0.0, 0.0, 0.0, +1.0);
}

//-----------------------------------------------------------------------------
void WorldView::viewToWorldRight()
{
  QTE_D();
  d->setView(+1.0, 0.0, 0.0, 0.0, 0.0, +1.0);
}

//-----------------------------------------------------------------------------
void WorldView::viewToWorldFront()
{
  QTE_D();
  d->setView(0.0, -1.0, 0.0, 0.0, 0.0, +1.0);
}

//-----------------------------------------------------------------------------
void WorldView::viewToWorldBack()
{
  QTE_D();
  d->setView(0.0, +1.0, 0.0, 0.0, 0.0, +1.0);
}

//-----------------------------------------------------------------------------
void WorldView::setPerspective(bool perspective)
{
  QTE_D();

  d->renderer->GetActiveCamera()->SetParallelProjection(!perspective);
  d->UI.renderWidget->update();
}

//-----------------------------------------------------------------------------
void WorldView::updateCameras()
{
  QTE_D();

  if (d->cameraRepDirty)
  {
    d->cameraRep->Update();
    d->UI.renderWidget->update();
    d->cameraRepDirty = false;
  }
}

//-----------------------------------------------------------------------------
void WorldView::updateScale()
{
  QTE_D();

  if (d->scaleDirty)
  {
    // Make sure the cameras are updated so that we will get the correct bounds
    // for them
    this->updateCameras();

    // Determine a base scale factor for the camera frustums... for now, using
    // the diagonal of the extents of the landmarks and camera centers
    vtkBoundingBox bbox;

    // Add landmarks
    foreach (auto const actor, d->landmarkActors)
    {
      bbox.AddBounds(actor->GetBounds());
    }

    // Update ground plane scale
    auto const groundScale =
      1.5 * qMax(qMax(qAbs(bbox.GetBound(0)), qAbs(bbox.GetBound(1))),
                 qMax(qAbs(bbox.GetBound(2)), qAbs(bbox.GetBound(3))));

    d->groundPlane->SetOrigin(-groundScale, -groundScale, 0.0);
    d->groundPlane->SetPoint1(+groundScale, -groundScale, 0.0);
    d->groundPlane->SetPoint2(-groundScale, +groundScale, 0.0);
    d->groundPlane->Modified();

    // Add camera centers
    bbox.AddBounds(d->cameraRep->GetPathActor()->GetBounds());

    // Compute base scale (20% of scale factor)
    auto const cameraScale = 0.2 * bbox.GetDiagonalLength();

    // Update camera scale
    d->cameraOptions->setBaseCameraScale(cameraScale);

    d->scaleDirty = false;
  }
}
