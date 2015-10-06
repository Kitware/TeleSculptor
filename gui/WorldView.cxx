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

//-----------------------------------------------------------------------------
class WorldViewPrivate
{
public:
  void addPopupMenu(QAction* action, QMenu* menu);

  double baseActiveCameraFrustumLength;

  Ui::WorldView UI;
  Am::WorldView AM;

  vtkNew<vtkRenderer> renderer;
  vtkNew<vtkRenderWindow> renderWindow;

  vtkNew<vtkMaptkCameraRepresentation> cameraRep;
  vtkNew<vtkActorCollection> landmarkActors;
};

QTE_IMPLEMENT_D_FUNC(WorldView)

//-----------------------------------------------------------------------------
void WorldViewPrivate::addPopupMenu(QAction* action, QMenu* menu)
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
  d->addPopupMenu(d->UI.actionViewReset, viewMenu);

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

  d->baseActiveCameraFrustumLength = 15;

  d->cameraRep->SetActiveCameraRepLength(d->baseActiveCameraFrustumLength);
  d->cameraRep->
    SetNonActiveCameraRepLength(0.20 * d->baseActiveCameraFrustumLength);

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

  // TODO coalesce updates to camera representation
  d->cameraRep->Update();
  d->UI.renderWidget->update();
}

//-----------------------------------------------------------------------------
void WorldView::setActiveCamera(vtkMaptkCamera* camera)
{
  QTE_D();

  d->cameraRep->SetActiveCamera(camera);

  // TODO coalesce updates to camera representation
  d->cameraRep->Update();
  d->UI.renderWidget->update();
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
void WorldView::computeCameraScale()
{
  QTE_D();

  // Compute base scale for the active camera as 10% of the diagonal of the
  // the bounds defined by the landmarks and the camera centers.
  vtkBoundingBox bbox;

  d->landmarkActors->InitTraversal();
  while (auto const actor = d->landmarkActors->GetNextActor())
  {
    bbox.AddBounds(actor->GetBounds());
  }

  bbox.AddBounds(d->cameraRep->GetPathActor()->GetBounds());

  d->baseActiveCameraFrustumLength = 0.1 * bbox.GetDiagonalLength();

  d->cameraRep->SetActiveCameraRepLength(d->baseActiveCameraFrustumLength);
  d->cameraRep->
    SetNonActiveCameraRepLength(0.20 * d->baseActiveCameraFrustumLength);
  d->cameraRep->Update();
}
