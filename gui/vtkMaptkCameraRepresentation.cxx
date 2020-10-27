/*ckwg +29
 * Copyright 2016-2020 by Kitware, Inc.
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

#include "vtkMaptkCameraRepresentation.h"

#include "arrows/vtk/vtkKwiverCamera.h"
#include <vital/types/vector.h>


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

#include <map>
#include <unordered_map>
#include <unordered_set>

vtkStandardNewMacro(vtkMaptkCameraRepresentation);

using kwiver::vital::vector_3d;

namespace // anonymous
{

typedef vtkSmartPointer<vtkPolyData> PolyDataPointer;

//-----------------------------------------------------------------------------
void BuildCameraFrustum(
  vtkCamera* camera, double farClipDistance, vtkPolyData* polyData)
{
  // Build frustum from camera data
  vtkNew<vtkPlanes> planes;
  double planeCoeffs[24];
  if (kwiver::arrows::vtk::vtkKwiverCamera::SafeDownCast(camera))
  {
    vtkNew<kwiver::arrows::vtk::vtkKwiverCamera> tempCamera;
    tempCamera->DeepCopy(kwiver::arrows::vtk::vtkKwiverCamera::SafeDownCast(camera));
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
  vector_3d points[4];
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
  auto const newPoint = vector_3d(points[2] + points[3] - center);

  // Insert new point and new face
  vtkIdType const newIndex = frustumPoints->InsertNextPoint(newPoint.data());
  vtkIdType const pts[3] = {2, 3, newIndex};
  polyData->GetPolys()->InsertNextCell(3, pts);
}

//-----------------------------------------------------------------------------
void SetInputs(vtkAppendPolyData* apd, std::unordered_set<vtkPolyData*> inputs)
{
  // Walk the list of existing inputs, removing any that aren't in the new list
  auto i = apd->GetNumberOfInputConnections(0);
  while (i--)
  {
    auto const input = apd->GetInput(i);
    auto const ii = inputs.find(input);
    if (ii != inputs.end())
    {
      inputs.erase(ii);
    }
    else
    {
      apd->RemoveInputConnection(0, apd->GetInputConnection(0, i));
    }
  }

  // Anything still in the new inputs list is missing from the existing inputs,
  // so now we can add those
  for (auto const input : inputs)
  {
    apd->AddInputData(input);
  }
}

} // namespace <anonymous>

//-----------------------------------------------------------------------------
class vtkMaptkCameraRepresentation::vtkInternal
{
public:
  vtkCamera* NextCamera();

  std::map<int, vtkCamera*> Cameras;

  vtkNew<vtkPolyData> ActivePolyData;
  vtkNew<vtkAppendPolyData> NonActiveAppendPolyData;

  vtkNew<vtkPolyData> DummyPolyData;

  vtkNew<vtkPolyData> PathPolyData;

  std::unordered_map<vtkCamera*, PolyDataPointer> CameraNonActivePolyData;

  vtkCamera* LastActiveCamera;
  double LastNonActiveCameraRepLength;

  bool PathNeedsUpdate;
};


//-----------------------------------------------------------------------------
vtkMaptkCameraRepresentation::vtkMaptkCameraRepresentation()
  : Internal(new vtkInternal)
{
  this->ActiveCameraRepLength = 15.0;
  this->NonActiveCameraRepLength = 4.0;
  this->DisplayDensity = 1;

  this->ActiveCamera = 0;

  this->Internal->LastActiveCamera = 0;
  this->Internal->LastNonActiveCameraRepLength = -1.0;
  this->Internal->PathNeedsUpdate = false;

  // Set up camera actors and data
  vtkNew<vtkPolyDataMapper> activeCameraMapper;
  activeCameraMapper->SetInputData(this->Internal->ActivePolyData.GetPointer());

  this->ActiveActor = vtkActor::New();
  this->ActiveActor->SetMapper(activeCameraMapper.GetPointer());
  this->ActiveActor->GetProperty()->SetRepresentationToWireframe();
  this->ActiveActor->GetProperty()->SetLighting(false);
  this->ActiveActor->PickableOff();

  this->Internal->NonActiveAppendPolyData->AddInputData(
    this->Internal->DummyPolyData.GetPointer());

  vtkNew<vtkPolyDataMapper> nonActiveMapper;
  nonActiveMapper->SetInputConnection(
    this->Internal->NonActiveAppendPolyData->GetOutputPort());

  this->NonActiveActor = vtkActor::New();
  this->NonActiveActor->SetMapper(nonActiveMapper.GetPointer());
  this->NonActiveActor->GetProperty()->SetRepresentationToWireframe();
  this->NonActiveActor->PickableOff();

  // Set up path actor and data
  vtkNew<vtkPoints> points;
  vtkNew<vtkCellArray> lines;

  this->Internal->PathPolyData->SetPoints(points.GetPointer());
  this->Internal->PathPolyData->SetLines(lines.GetPointer());

  vtkNew<vtkPolyDataMapper> pathMapper;
  pathMapper->SetInputData(this->Internal->PathPolyData.GetPointer());

  this->PathActor = vtkActor::New();
  this->PathActor->SetMapper(pathMapper.GetPointer());
  this->PathActor->PickableOff();
}

//-----------------------------------------------------------------------------
vtkMaptkCameraRepresentation::~vtkMaptkCameraRepresentation()
{
  this->ActiveActor->Delete();
  this->NonActiveActor->Delete();
  this->PathActor->Delete();
  for(auto const camData : this->Internal->Cameras)
  {
    camData.second->UnRegister(this);
  }
}

//-----------------------------------------------------------------------------
void vtkMaptkCameraRepresentation::AddCamera(int id, vtkCamera* camera)
{
  // Don't allow null or duplicate entries
  if (!camera || this->Internal->Cameras.count(id) > 0)
  {
    return;
  }

  this->Internal->Cameras[id] = camera;
  camera->Register(this);
  this->Internal->PathNeedsUpdate = true;
  this->Modified();
}

//-----------------------------------------------------------------------------
void vtkMaptkCameraRepresentation::RemoveCamera(int id)
{
  // If item isn't present, do nothing
  auto camIter = this->Internal->Cameras.find(id);
  if (camIter == this->Internal->Cameras.end())
  {
    return;
  }

  // Remove camera from non-active polydata collection
  auto const pd = this->Internal->CameraNonActivePolyData.find(camIter->second);
  if (pd != this->Internal->CameraNonActivePolyData.end())
  {
    this->Internal->NonActiveAppendPolyData->RemoveInputData(pd->second);
    this->Internal->CameraNonActivePolyData.erase(pd);
  }

  if (this->ActiveCamera == camIter->second)
  {
    this->ActiveCamera = 0;
  }

  camIter->second->UnRegister(this);
  this->Internal->Cameras.erase(camIter);
  this->Internal->PathNeedsUpdate = true;
  this->Modified();
}

//-----------------------------------------------------------------------------
void vtkMaptkCameraRepresentation::CamerasModified()
{
  this->Internal->PathNeedsUpdate = true;
  this->Internal->CameraNonActivePolyData.clear();
  this->Modified();
}

//-----------------------------------------------------------------------------
void vtkMaptkCameraRepresentation::SetActiveCamera(int id)
{
  auto camIter = this->Internal->Cameras.find(id);
  if (camIter == this->Internal->Cameras.end())
  {
    this->ActiveCamera = nullptr;
    return;
  }
  if (this->ActiveCamera != camIter->second)
  {
    this->ActiveCamera = camIter->second;
    this->Modified();
  }
}

//-----------------------------------------------------------------------------
void vtkMaptkCameraRepresentation::Update()
{
  if (this->Internal->LastNonActiveCameraRepLength !=
      this->NonActiveCameraRepLength)
  {
    // If the non-active camera length has changed, the old polydata's are not
    // useful, and every camera needs to be updated
    this->Internal->CameraNonActivePolyData.clear();
  }
  this->Internal->LastNonActiveCameraRepLength = this->NonActiveCameraRepLength;

  // (Re)build non-active cameras representation and build polydata for any
  // cameras that are missing
  std::unordered_set<vtkPolyData*> nonActivePolyData;
  int skipCount = 0;
  for(auto const camData : this->Internal->Cameras)
  {
    if (!((skipCount++) % this->DisplayDensity))
    {
      if (camData.second != this->ActiveCamera)
      {
        auto& pd = this->Internal->CameraNonActivePolyData[camData.second];
        if (!pd)
        {
          // If we don't already have polydata for this non-active camera,
          // build it now
          pd = PolyDataPointer::New();
          BuildCameraFrustum(camData.second, this->NonActiveCameraRepLength, pd);
        }
        nonActivePolyData.insert(pd);
      }
    }
  }

  if (nonActivePolyData.empty())
  {
    // vtkAppendPolyData complains if it has no inputs, so if there aren't any
    // non-active cameras, add the dummy polydata
    nonActivePolyData.insert(this->Internal->DummyPolyData.GetPointer());
  }

  SetInputs(this->Internal->NonActiveAppendPolyData.GetPointer(),
            nonActivePolyData);

  // (Re)build active camera representation if needed
  if (!this->ActiveCamera)
  {
    this->Internal->ActivePolyData->Reset();
    this->Internal->ActivePolyData->Modified();
  }
  else if (this->ActiveCamera != this->Internal->LastActiveCamera)
  {
    this->Internal->ActivePolyData->Reset();
    BuildCameraFrustum(this->ActiveCamera, this->ActiveCameraRepLength,
                       this->Internal->ActivePolyData.GetPointer());
  }

  // Path Actor
  if (this->Internal->PathNeedsUpdate)
  {
    this->Internal->PathPolyData->Reset();

    vtkNew<vtkPoints> points;
    this->Internal->PathPolyData->SetPoints(points.GetPointer());
    points->Allocate(this->Internal->Cameras.size());

    vtkNew<vtkCellArray> lines;
    this->Internal->PathPolyData->SetLines(lines.GetPointer());
    lines->InsertNextCell(static_cast<int>(this->Internal->Cameras.size()));

    for(auto const& camData : this->Internal->Cameras)
    {
      double position[3];
      camData.second->GetPosition(position);
      lines->InsertCellPoint(points->InsertNextPoint(position));
    }
    this->Internal->PathPolyData->Modified();
    this->Internal->PathNeedsUpdate = false;
  }
}

//-----------------------------------------------------------------------------
void vtkMaptkCameraRepresentation::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);

  os << indent << "Number Of Cameras: "
     << this->Internal->Cameras.size() << endl;
  os << indent << "ActiveCameraRepLength: "
     << this->ActiveCameraRepLength << endl;
  os << indent << "NonActiveCameraRepLength: "
     << this->NonActiveCameraRepLength << endl;
  os << indent << "DisplayDensity: "
     << this->DisplayDensity << endl;
}
