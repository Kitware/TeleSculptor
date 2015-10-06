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
#include "vtkMaptkCamera.h"
#include "vtkMaptkCameraRepresentation.h"

#include <maptk/camera.h>
#include <maptk/landmark_map.h>

#include <vtkActorCollection.h>
#include <vtkAppendPolyData.h>
#include <vtkBoundingBox.h>
#include <vtkCamera.h>
#include <vtkCellArray.h>
#include <vtkFrustumSource.h>
#include <vtkMath.h>
#include <vtkNew.h>
#include <vtkPlanes.h>
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
  WorldViewPrivate() : cameraRepDirty(false), cameraScaleDirty(false) {}

  void setPopup(QAction* action, QMenu* menu);
  void setPopup(QAction* action, QWidget* widget);

  void updateCameras(WorldView*);
  void updateCameraScale(WorldView*);

  Ui::WorldView UI;
  Am::WorldView AM;

  vtkNew<vtkRenderer> renderer;
  vtkNew<vtkRenderWindow> renderWindow;

  vtkNew<vtkMaptkCameraRepresentation> cameraRep;
  vtkNew<vtkActorCollection> landmarkActors;

  bool cameraRepDirty;
  bool cameraScaleDirty;
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
void WorldViewPrivate::updateCameras(WorldView* q)
{
  if (!this->cameraRepDirty)
  {
    this->cameraRepDirty = true;
    QMetaObject::invokeMethod(q, "updateCameras");
  }
}

//-----------------------------------------------------------------------------
void WorldViewPrivate::updateCameraScale(WorldView* q)
{
  if (!this->cameraScaleDirty)
  {
    this->cameraScaleDirty = true;
    QMetaObject::invokeMethod(q, "updateCameraScale");
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
  d->setPopup(d->UI.actionViewReset, viewMenu);

  auto const cameraOptions = new CameraOptions(this);
  cameraOptions->setRepresentation(d->cameraRep.GetPointer());
  d->setPopup(d->UI.actionShowCameras, cameraOptions);

  // Connect actions
  this->addAction(d->UI.actionViewReset);
  this->addAction(d->UI.actionViewResetLandmarks);

  connect(d->UI.actionViewReset, SIGNAL(triggered()),
          this, SLOT(resetView()));
  connect(d->UI.actionViewResetLandmarks, SIGNAL(triggered()),
          this, SLOT(resetViewToLandmarks()));

  connect(d->UI.actionShowCameras, SIGNAL(toggled(bool)),
          this, SLOT(setCamerasVisible(bool)));
  connect(d->UI.actionShowLandmarks, SIGNAL(toggled(bool)),
          this, SLOT(setLandmarksVisible(bool)));

  // Set up render pipeline
  d->renderer->SetBackground(0, 0, 0);
  d->renderWindow->AddRenderer(d->renderer.GetPointer());
  d->UI.renderWidget->SetRenderWindow(d->renderWindow.GetPointer());

  d->renderer->AddActor(d->cameraRep->GetNonActiveActor());
  d->renderer->AddActor(d->cameraRep->GetActiveActor());
  d->renderer->AddActor(d->cameraRep->GetPathActor());
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

  d->updateCameraScale(this);
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
  actor->GetProperty()->SetPointSize(2);
  actor->GetProperty()->SetColor(1.0, 0.0, 1.0);

  d->renderer->AddActor(actor.GetPointer());

  d->landmarkActors->AddItem(actor.GetPointer());

  d->updateCameraScale(this);
}

//-----------------------------------------------------------------------------
void WorldView::setCamerasVisible(bool state)
{
  QTE_D();

  d->cameraRep->GetPathActor()->SetVisibility(state);
  d->cameraRep->GetActiveActor()->SetVisibility(state);
  d->cameraRep->GetNonActiveActor()->SetVisibility(state);

  d->UI.renderWidget->update();
}

//-----------------------------------------------------------------------------
void WorldView::setLandmarksVisible(bool state)
{
  QTE_D();

  d->landmarkActors->InitTraversal();
  while (auto const actor = d->landmarkActors->GetNextActor())
  {
    actor->SetVisibility(state);
  }

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

  d->landmarkActors->InitTraversal();
  while (auto const actor = d->landmarkActors->GetNextActor())
  {
    bbox.AddBounds(actor->GetBounds());
  }

  double bounds[6];
  bbox.GetBounds(bounds);
  d->renderer->ResetCamera(bounds);

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
void WorldView::updateCameraScale()
{
  QTE_D();

  if (d->cameraScaleDirty)
  {
    // Determine a base scale factor for the camera frustums... for now, using
    // the diagonal of the extents of the landmarks and camera centers
    vtkBoundingBox bbox;

    // Add landmarks
    d->landmarkActors->InitTraversal();
    while (auto const actor = d->landmarkActors->GetNextActor())
    {
      bbox.AddBounds(actor->GetBounds());
    }

    // Add camera centers
    bbox.AddBounds(d->cameraRep->GetPathActor()->GetBounds());

    // Compute base scale (10% of scale factor)
    auto const baseScale = 0.1 * bbox.GetDiagonalLength();

    d->cameraRep->SetActiveCameraRepLength(baseScale);
    d->cameraRep->SetNonActiveCameraRepLength(baseScale);
    d->updateCameras(this);
  }
}
