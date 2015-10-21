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

#include "vtkMaptkCameraRepresentation.h"

#include "vtkMaptkCamera.h"

#include <maptk/vector.h>

#include <vtkActor.h>
#include <vtkAppendPolyData.h>
#include <vtkCellArray.h>
#include <vtkCollection.h>
#include <vtkFrustumSource.h>
#include <vtkNew.h>
#include <vtkObjectFactory.h>
#include <vtkPlanes.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>

vtkStandardNewMacro(vtkMaptkCameraRepresentation);

namespace // anonymous
{

//-----------------------------------------------------------------------------
void BuildCameraFrustum(
  vtkCamera* camera, double farClipDistance, vtkPolyData* polyData)
{
  // Build frustum from camera data
  vtkNew<vtkPlanes> planes;
  double planeCoeffs[24];
  if (vtkMaptkCamera::SafeDownCast(camera))
  {
    vtkNew<vtkMaptkCamera> tempCamera;
    tempCamera->DeepCopy(vtkMaptkCamera::SafeDownCast(camera));
    tempCamera->SetClippingRange(0.01, farClipDistance);
    tempCamera->GetFrustumPlanes(planeCoeffs);
  }
  else
  {
    vtkNew<vtkCamera> tempCamera;
    tempCamera->DeepCopy(camera);
    tempCamera->SetClippingRange(0.01, farClipDistance);
    // use aspect of 1.0 if not a Maptk camera
    tempCamera->GetFrustumPlanes(1.0, planeCoeffs);
  }
  planes->SetFrustumPlanes(planeCoeffs);

  vtkNew<vtkFrustumSource> frustum;
  frustum->SetPlanes(planes.GetPointer());
  frustum->SetShowLines(false);
  frustum->Update();

  // Make a copy of the frustum mesh so we can modify it
  polyData->DeepCopy(frustum->GetOutput());
  auto const frustumPoints = polyData->GetPoints();

  // Add a polygon to indicate the up direction (the far plane uses points
  // 0, 1, 2, and 3, with 2 and 3 on the top; we use those with the center of
  // the far polygon to compute a point "above" the far face to form a triangle
  // like the roof of a "house")
  //
  // TODO vtkFrustumSource indicates that this is actually the near plane, but
  //      appears to be wrong - need to verify
  maptk::vector_3d points[4];
  frustumPoints->GetPoint(0, points[0].data());
  frustumPoints->GetPoint(1, points[1].data());
  frustumPoints->GetPoint(2, points[2].data());
  frustumPoints->GetPoint(3, points[3].data());

  // Compute new point:
  //   center = (p0 + p1 + p2 + p3) / 4.0
  //   top = (p2 + p3) / 2.0
  //   new = top + (top - center)
  //       = p2 + p3 - center
  auto const center = 0.25 * (points[0] + points[1] + points[2] + points[3]);
  auto const newPoint = maptk::vector_3d(points[2] + points[3] - center);

  // Insert new point and new face
  vtkIdType const newIndex = frustumPoints->InsertNextPoint(newPoint.data());
  vtkIdType const pts[3] = {2, 3, newIndex};
  polyData->GetPolys()->InsertNextCell(3, pts);
}

} // namespace <anonymous>

//-----------------------------------------------------------------------------
class vtkMaptkCameraRepresentation::vtkInternal
{
public:
  vtkCamera* NextCamera();

  vtkNew<vtkCollection> Cameras;

  vtkNew<vtkPolyData> ActivePolyData;
  vtkNew<vtkAppendPolyData> NonActiveAppendPolyData;

  vtkNew<vtkPolyData> DummyPolyData;

  vtkNew<vtkPolyData> PathPolyData;
};

//-----------------------------------------------------------------------------
vtkCamera* vtkMaptkCameraRepresentation::vtkInternal::NextCamera()
{
  return vtkCamera::SafeDownCast(this->Cameras->GetNextItemAsObject());
}

//-----------------------------------------------------------------------------
vtkMaptkCameraRepresentation::vtkMaptkCameraRepresentation()
  : Internal(new vtkInternal)
{
  this->ActiveCameraRepLength = 15.0;
  this->NonActiveCameraRepLength = 4.0;
  this->DisplayDensity = 1;

  this->ActiveCamera = 0;

  // Set up camera actors and data
  vtkNew<vtkPolyDataMapper> activeCameraMapper;
  activeCameraMapper->SetInputData(this->Internal->ActivePolyData.GetPointer());

  this->ActiveActor = vtkActor::New();
  this->ActiveActor->SetMapper(activeCameraMapper.GetPointer());
  this->ActiveActor->GetProperty()->SetRepresentationToWireframe();
  this->ActiveActor->GetProperty()->SetLighting(false);

  this->Internal->NonActiveAppendPolyData->AddInputData(
    this->Internal->DummyPolyData.GetPointer());

  vtkNew<vtkPolyDataMapper> nonActiveMapper;
  nonActiveMapper->SetInputConnection(
    this->Internal->NonActiveAppendPolyData->GetOutputPort());

  this->NonActiveActor = vtkActor::New();
  this->NonActiveActor->SetMapper(nonActiveMapper.GetPointer());
  this->NonActiveActor->GetProperty()->SetRepresentationToWireframe();

  // Set up path actor and data
  vtkNew<vtkPoints> points;
  vtkNew<vtkCellArray> lines;

  this->Internal->PathPolyData->SetPoints(points.GetPointer());
  this->Internal->PathPolyData->SetLines(lines.GetPointer());

  vtkNew<vtkPolyDataMapper> pathMapper;
  pathMapper->SetInputData(this->Internal->PathPolyData.GetPointer());

  this->PathActor = vtkActor::New();
  this->PathActor->SetMapper(pathMapper.GetPointer());
}

//-----------------------------------------------------------------------------
vtkMaptkCameraRepresentation::~vtkMaptkCameraRepresentation()
{
  this->ActiveActor->Delete();
  this->NonActiveActor->Delete();
  this->PathActor->Delete();
}

//-----------------------------------------------------------------------------
void vtkMaptkCameraRepresentation::AddCamera(vtkCamera* camera)
{
  // Don't allow duplicate entries
  if (this->Internal->Cameras->IsItemPresent(camera))
  {
    return;
  }

  this->Internal->Cameras->AddItem(camera);
  this->Modified();
}

//-----------------------------------------------------------------------------
void vtkMaptkCameraRepresentation::RemoveCamera(vtkCamera* camera)
{
  // If item isn't present, do nothing
  if (!this->Internal->Cameras->IsItemPresent(camera))
  {
    return;
  }

  this->Internal->Cameras->RemoveItem(camera);
  if (this->ActiveCamera == camera)
  {
    this->ActiveCamera = 0;
  }

  this->Modified();
}

//-----------------------------------------------------------------------------
void vtkMaptkCameraRepresentation::SetActiveCamera(vtkCamera* camera)
{
  if (this->ActiveCamera == camera)
  {
    return;
  }

  // Make sure the camera is in the list
  this->AddCamera(camera);

  this->ActiveCamera = camera;
  this->Modified();
}

//-----------------------------------------------------------------------------
void vtkMaptkCameraRepresentation::Update()
{
  // TODO - Add modified time test

  // Build non-active cameras representation
  this->Internal->NonActiveAppendPolyData->RemoveAllInputs();
  this->Internal->NonActiveAppendPolyData->AddInputData(
    this->Internal->DummyPolyData.GetPointer());

  int skipCount = 0;
  this->Internal->Cameras->InitTraversal();
  while (auto const camera = this->Internal->NextCamera())
  {
    if (!((skipCount++) % this->DisplayDensity))
    {
      if (camera != this->ActiveCamera)
      {
        vtkNew<vtkPolyData> polyData;
        BuildCameraFrustum(camera, this->NonActiveCameraRepLength,
                           polyData.GetPointer());
        this->Internal->NonActiveAppendPolyData->AddInputData(
          polyData.GetPointer());
      }
    }
  }

  // Build active camera representation
  this->Internal->ActivePolyData->Reset();
  if (this->ActiveCamera)
  {
    BuildCameraFrustum(this->ActiveCamera, this->ActiveCameraRepLength,
                       this->Internal->ActivePolyData.GetPointer());
  }

  // Path Actor
  this->Internal->PathPolyData->Reset();
  this->Internal->PathPolyData->Modified();
  vtkPoints* points = this->Internal->PathPolyData->GetPoints();
  points->Allocate(this->Internal->Cameras->GetNumberOfItems());
  vtkCellArray* lines = this->Internal->PathPolyData->GetLines();
  lines->InsertNextCell(this->Internal->Cameras->GetNumberOfItems());

  this->Internal->Cameras->InitTraversal();
  while (auto const camera = this->Internal->NextCamera())
  {
    double position[3];
    camera->GetPosition(position);
    lines->InsertCellPoint(points->InsertNextPoint(position));
  }
}

//-----------------------------------------------------------------------------
void vtkMaptkCameraRepresentation::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);

  os << indent << "Number Of Cameras: "
     << this->Internal->Cameras->GetNumberOfItems() << endl;
  os << indent << "ActiveCameraRepLength: "
     << this->ActiveCameraRepLength << endl;
  os << indent << "NonActiveCameraRepLength: "
     << this->NonActiveCameraRepLength << endl;
  os << indent << "DisplayDensity: "
     << this->DisplayDensity << endl;
}
