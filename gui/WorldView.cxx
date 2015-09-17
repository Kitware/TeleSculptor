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

#include <maptk/camera.h>
#include <maptk/landmark_map.h>

#include <vtkAppendPolyData.h>
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

namespace // anonymous
{

//-----------------------------------------------------------------------------
struct Camera {
  maptk::vector_3d center; // Camera position
  maptk::vector_3d view; // Direction vector of camera view
  maptk::vector_3d up; // Direction vector of camera up axis
  double fov; // Camera field-of-view angle, in degrees
  double aspect; // Camera aspect ratio (image width / image height)
};

//-----------------------------------------------------------------------------
double computeFov(double width, double length)
{
  return vtkMath::DegreesFromRadians(2.0 * atan(0.5 * width / length));
}

//-----------------------------------------------------------------------------
Camera buildCamera(maptk::camera const& camera, QSizeF const& frameSize)
{
  Camera out;

  // Get camera parameters
  auto const& ci = camera.intrinsics();
  auto const pixelAspect = ci.aspect_ratio();
  auto const focalLength = ci.focal_length();

  out.aspect = pixelAspect * frameSize.width() / frameSize.height();
  out.fov = computeFov(frameSize.height(), focalLength);

  // Compute camera vectors from matrix
  auto const& rotationMatrix =
    camera.rotation().quaternion().toRotationMatrix();

  out.up = -rotationMatrix.row(1).transpose();
  out.view = rotationMatrix.row(2).transpose();
  out.center = camera.center();

  return out;
}

//-----------------------------------------------------------------------------
void buildFrustum(vtkPlanes* out, Camera const& c)
{
  auto const depth = 15.0; // TODO make configurable or something
  auto const& focus = c.center + (c.view * depth / c.view.norm());

  vtkNew<vtkCamera> camera;

  camera->SetPosition(c.center[0], c.center[1], c.center[2]);
  camera->SetFocalPoint(focus[0], focus[1], focus[2]);
  camera->SetViewUp(c.up[0], c.up[1], c.up[2]);
  camera->SetViewAngle(c.fov);
  camera->SetClippingRange(0.01, depth);

  double planeCoeffs[24];
  camera->GetFrustumPlanes(c.aspect, planeCoeffs);
  out->SetFrustumPlanes(planeCoeffs);
}

} // namespace <anonymous>

//-----------------------------------------------------------------------------
class WorldViewPrivate
{
public:
  vtkNew<vtkRenderer> renderer;
  vtkNew<vtkRenderWindow> renderWindow;

  vtkNew<vtkAppendPolyData> cameraData;
  vtkNew<vtkPolyData> dummyData;
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

  // Set up actor for camera frustums
  vtkNew<vtkActor> cameraActor;
  vtkNew<vtkPolyDataMapper> cameraMapper;
  cameraMapper->SetInputConnection(d->cameraData->GetOutputPort());
  cameraActor->SetMapper(cameraMapper.GetPointer());
  cameraActor->GetProperty()->SetRepresentationToWireframe();

  d->renderer->AddActor(cameraActor.GetPointer());

  // Add some dummy "data" to the camera data collection so VTK doesn't
  // complain about non-optional input port connections
  d->cameraData->AddInputData(d->dummyData.GetPointer());
}

//-----------------------------------------------------------------------------
WorldView::~WorldView()
{
}

//-----------------------------------------------------------------------------
void WorldView::addCamera(
  int id, maptk::camera const& camera, QSize const& frameSize)
{
  Q_UNUSED(id)

  QTE_D();

  // Build frustum from camera data
  vtkNew<vtkPlanes> planes;
  buildFrustum(planes.GetPointer(), buildCamera(camera, frameSize));

  vtkNew<vtkFrustumSource> frustum;
  frustum->SetPlanes(planes.GetPointer());
  frustum->SetShowLines(false);
  frustum->Update();

  // Make a copy of the frustum mesh so we can modify it
  vtkNew<vtkPolyData> polyData;
  polyData->DeepCopy(frustum->GetOutput());
  auto frustumPoints = polyData->GetPoints();

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
  vtkIdType newIndex = frustumPoints->InsertNextPoint(newPoint.data());
  vtkCellArray* polys = polyData->GetPolys();
  vtkIdType pts[3] = { 2, 3, newIndex };
  polys->InsertNextCell(3, pts);

  // Add mesh to camera meshes
  d->cameraData->AddInputData(polyData.GetPointer());
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
}
