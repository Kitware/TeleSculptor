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

//-----------------------------------------------------------------------------
class WorldViewPrivate
{
public:
  vtkNew<vtkRenderer> renderer;
  vtkNew<vtkRenderWindow> renderWindow;

  vtkNew<vtkMaptkCameraRepresentation> camerasRep;
  vtkNew<vtkActorCollection> landmarkActors;
};

QTE_IMPLEMENT_D_FUNC(WorldView)

//-----------------------------------------------------------------------------
WorldView::WorldView(QWidget* parent, Qt::WindowFlags flags)
  : QVTKWidget(parent, flags), d_ptr(new WorldViewPrivate)
{
  QTE_D();

  // Set up render pipeline
  d->renderer->SetBackground(0, 0, 0);
  d->renderWindow->AddRenderer(d->renderer.GetPointer());
  this->SetRenderWindow(d->renderWindow.GetPointer());

  d->renderer->AddActor(d->camerasRep->GetNonActiveActor());
  d->renderer->AddActor(d->camerasRep->GetActiveActor());
  d->renderer->AddActor(d->camerasRep->GetPathActor());
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

  d->camerasRep->AddCamera(camera);
}

//-----------------------------------------------------------------------------
void WorldView::updateCameraActors()
{
  QTE_D();

  d->camerasRep->Update();
  d->renderWindow->Render();
}

//-----------------------------------------------------------------------------
void WorldView::setActiveCamera(vtkMaptkCamera* camera)
{
  QTE_D();

  d->camerasRep->SetActiveCamera(camera);
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
  d->renderer->AddActor(actor.GetPointer());

  d->landmarkActors->AddItem(actor.GetPointer());
}

//-----------------------------------------------------------------------------
void WorldView::resetViewToLandmarks()
{
  QTE_D();

  vtkBoundingBox bbox;

  d->landmarkActors->InitTraversal();
  vtkActor* actor;
  while ((actor = d->landmarkActors->GetNextActor()))
  {
    bbox.AddBounds(actor->GetBounds());
  }

  double bounds[6];
  bbox.GetBounds(bounds);
  d->renderer->ResetCamera(bounds);
  d->renderWindow->Render();
}
