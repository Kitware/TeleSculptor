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

// TeleSculptor includes
#include "vtkMaptkAxesActor.h"

// VTK includes
#include <vtkActor.h>
#include <vtkCylinderSource.h>
#include <vtkDiskSource.h>
#include <vtkObjectFactory.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkSphereSource.h>
#include <vtkTransform.h>

vtkStandardNewMacro(vtkMaptkAxesActor);

//-----------------------------------------------------------------------------
vtkMaptkAxesActor::vtkMaptkAxesActor()
{
  this->CylinderSource = vtkCylinderSource::New();
  this->CylinderSource->SetRadius(0.04);
  this->CylinderSource->SetResolution(20);
  this->CylinderSource->SetHeight(1.0);

  vtkPolyDataMapper* shaftMapper = vtkPolyDataMapper::New();
  shaftMapper->SetInputConnection(this->CylinderSource->GetOutputPort());
  this->ZAxisShaft = vtkActor::New();
  this->ZAxisShaft->SetMapper(shaftMapper);
  this->ZAxisShaft->GetProperty()->SetColor(0, 0, 1);
  this->XAxisShaft = vtkActor::New();
  this->XAxisShaft->SetMapper(shaftMapper);
  this->XAxisShaft->GetProperty()->SetColor(1, 0, 0);
  this->YAxisShaft = vtkActor::New();
  this->YAxisShaft->SetMapper(shaftMapper);
  this->YAxisShaft->GetProperty()->SetColor(0, 1, 0);
  shaftMapper->Delete();

  this->SphereSource = vtkSphereSource::New();
  this->SphereSource->SetThetaResolution(40);
  this->SphereSource->SetPhiResolution(40);
  this->SphereSource->SetRadius(0.5);

  vtkPolyDataMapper* tipMapper = vtkPolyDataMapper::New();
  tipMapper->SetInputConnection(this->SphereSource->GetOutputPort());
  this->ZAxisTip = vtkActor::New();
  this->ZAxisTip->SetMapper(tipMapper);
  this->ZAxisTip->GetProperty()->SetColor(0, 0, 1);
  this->YAxisTip = vtkActor::New();
  this->YAxisTip->SetMapper(tipMapper);
  this->YAxisTip->GetProperty()->SetColor(0, 1, 0);
  this->XAxisTip = vtkActor::New();
  this->XAxisTip->SetMapper(tipMapper);
  this->XAxisTip->GetProperty()->SetColor(1, 0, 0);
  tipMapper->Delete();

  this->DiskSource = vtkDiskSource::New();
  this->DiskSource->SetRadialResolution(3);
  this->DiskSource->SetCircumferentialResolution(50);
  this->DiskSource->SetInnerRadius(0.15);
  this->DiskSource->SetOuterRadius(1.0);
  vtkPolyDataMapper* diskMapper = vtkPolyDataMapper::New();
  diskMapper->SetInputConnection(this->DiskSource->GetOutputPort());
  this->XYPlaneDisk = vtkActor::New();
  this->XYPlaneDisk->SetMapper(diskMapper);
  this->XYPlaneDisk->GetProperty()->SetColor(0.8, 0.5, 0);
  this->XYPlaneDisk->GetProperty()->SetOpacity(0.2);
  diskMapper->Delete();
}

//-----------------------------------------------------------------------------
vtkMaptkAxesActor::~vtkMaptkAxesActor()
{
  this->CylinderSource->Delete();
  this->SphereSource->Delete();
  this->ZAxisShaft->Delete();
  this->ZAxisTip->Delete();
  this->YAxisShaft->Delete();
  this->YAxisTip->Delete();
  this->XAxisShaft->Delete();
  this->XAxisTip->Delete();
  this->DiskSource->Delete();
  this->XYPlaneDisk->Delete();
}

//----------------------------------------------------------------------------
void vtkMaptkAxesActor::PrintSelf(ostream& os, vtkIndent indent)
{
  os << indent
     << "ZAxisVisibility = " << (this->ZAxisVisibility ? "True" : "False")
     << endl;
  os << indent
     << "XYPlaneVisibility = " << (this->XYPlaneVisibility ? "True" : "False")
     << endl;
  os << indent << "AxesLength = " << this->AxesLength << endl;
  this->Superclass::PrintSelf(os, indent);
}

//----------------------------------------------------------------------------
void vtkMaptkAxesActor::GetBounds(double bounds[6])
{
  const double* bds = this->GetBounds();
  bounds[0] = bds[0];
  bounds[1] = bds[1];
  bounds[2] = bds[2];
  bounds[3] = bds[3];
  bounds[4] = bds[4];
  bounds[5] = bds[5];
}

//----------------------------------------------------------------------------
// Get the bounds for this Actor as (Xmin,Xmax,Ymin,Ymax,Zmin,Zmax).
double* vtkMaptkAxesActor::GetBounds()
{
  double bounds[6];
  int i;

  this->ZAxisShaft->GetBounds(bounds);
  for (i = 0; i < 3; ++i)
  {
    this->Bounds[2 * i + 1] = (bounds[2 * i + 1] > this->Bounds[2 * i + 1])
      ? (bounds[2 * i + 1])
      : (this->Bounds[2 * i + 1]);
  }

  this->ZAxisTip->GetBounds(bounds);
  for (i = 0; i < 3; ++i)
  {
    this->Bounds[2 * i + 1] = (bounds[2 * i + 1] > this->Bounds[2 * i + 1])
      ? (bounds[2 * i + 1])
      : (this->Bounds[2 * i + 1]);
  }

  this->YAxisShaft->GetBounds(bounds);
  for (i = 0; i < 3; ++i)
  {
    this->Bounds[2 * i + 1] = (bounds[2 * i + 1] > this->Bounds[2 * i + 1])
      ? (bounds[2 * i + 1])
      : (this->Bounds[2 * i + 1]);
  }

  this->YAxisTip->GetBounds(bounds);
  for (i = 0; i < 3; ++i)
  {
    this->Bounds[2 * i + 1] = (bounds[2 * i + 1] > this->Bounds[2 * i + 1])
      ? (bounds[2 * i + 1])
      : (this->Bounds[2 * i + 1]);
  }

  this->XAxisShaft->GetBounds(bounds);
  for (i = 0; i < 3; ++i)
  {
    this->Bounds[2 * i + 1] = (bounds[2 * i + 1] > this->Bounds[2 * i + 1])
      ? (bounds[2 * i + 1])
      : (this->Bounds[2 * i + 1]);
  }

  this->XAxisTip->GetBounds(bounds);
  for (i = 0; i < 3; ++i)
  {
    this->Bounds[2 * i + 1] = (bounds[2 * i + 1] > this->Bounds[2 * i + 1])
      ? (bounds[2 * i + 1])
      : (this->Bounds[2 * i + 1]);
  }

  this->XYPlaneDisk->GetBounds(bounds);
  for (i = 0; i < 3; ++i)
  {
    this->Bounds[2 * i + 1] = (bounds[2 * i + 1] > this->Bounds[2 * i + 1])
      ? (bounds[2 * i + 1])
      : (this->Bounds[2 * i + 1]);
  }

  // We want this actor to rotate / re-center about the origin, so give it
  // the bounds it would have if the axes were symmetric.
  for (i = 0; i < 3; ++i)
  {
    this->Bounds[2 * i] = -this->Bounds[2 * i + 1];
  }

  return this->Bounds;
}

//----------------------------------------------------------------------------
void vtkMaptkAxesActor::UpdateProps()
{
  double pos[3];
  this->GetPosition(pos);

  if (this->GetUserTransform())
  {
    this->ZAxisShaft->SetUserTransform(nullptr);
    this->ZAxisTip->SetUserTransform(nullptr);
    this->YAxisShaft->SetUserTransform(nullptr);
    this->YAxisTip->SetUserTransform(nullptr);
    this->XAxisShaft->SetUserTransform(nullptr);
    this->XAxisTip->SetUserTransform(nullptr);
    this->XYPlaneDisk->SetUserTransform(nullptr);
  }

  this->ZAxisShaft->SetVisibility(this->GetZAxisVisibility());
  this->ZAxisTip->SetVisibility(this->GetZAxisVisibility());

  if (this->GetZAxisVisibility())
  {
    // 70% shaft
    double shaftLength = 0.7 * this->AxesLength;

    this->ZAxisShaft->SetScale(
      this->AxesLength, this->AxesLength, this->AxesLength);
    this->ZAxisShaft->SetPosition(pos[0], pos[1], pos[2] + shaftLength / 2);
    this->ZAxisShaft->SetOrientation(90, 0, 0);

    // 30% tip
    double tipLength = 0.3 * this->AxesLength;

    this->ZAxisTip->SetScale(tipLength, tipLength, tipLength);
    this->ZAxisTip->SetPosition(pos[0], pos[1], pos[2] + this->AxesLength);
    this->ZAxisTip->SetOrientation(90, 0, 0);
  }

  this->XYPlaneDisk->SetVisibility(this->GetXYPlaneVisibility());
  this->YAxisShaft->SetVisibility(this->GetXYPlaneVisibility());
  this->YAxisTip->SetVisibility(this->GetXYPlaneVisibility());
  this->XAxisShaft->SetVisibility(this->GetXYPlaneVisibility());
  this->XAxisTip->SetVisibility(this->GetXYPlaneVisibility());
  if (this->XYPlaneVisibility)
  {
    this->XYPlaneDisk->SetScale(
      this->AxesLength, this->AxesLength, this->AxesLength);
    this->XYPlaneDisk->SetPosition(pos[0], pos[1], pos[2]);
    this->XYPlaneDisk->SetOrientation(0, 0, 0);

    // 70% shaft
    double shaftLength = 0.7 * this->AxesLength;

    this->YAxisShaft->SetScale(
      this->AxesLength, this->AxesLength, this->AxesLength);
    this->YAxisShaft->SetPosition(pos[0], pos[1] + shaftLength / 2, pos[2]);
    this->YAxisShaft->SetOrientation(0, 0, 0);
    this->XAxisShaft->SetScale(
      this->AxesLength, this->AxesLength, this->AxesLength);
    this->XAxisShaft->SetPosition(pos[0] + shaftLength / 2, pos[1], pos[2]);
    this->XAxisShaft->SetOrientation(0, 0, -90);

    // 30% tip
    double tipLength = 0.3 * this->AxesLength;

    this->YAxisTip->SetScale(tipLength, tipLength, tipLength);
    this->YAxisTip->SetPosition(pos[0], pos[1] + this->AxesLength, pos[2]);
    this->YAxisTip->SetOrientation(0, 0, 0);
    this->XAxisTip->SetScale(tipLength, tipLength, tipLength);
    this->XAxisTip->SetPosition(pos[0] + this->AxesLength, pos[1], pos[2]);
    this->XAxisTip->SetOrientation(0, 0, -90);
  }
}

//----------------------------------------------------------------------------
int vtkMaptkAxesActor::RenderOpaqueGeometry(vtkViewport* vp)
{
  int renderedSomething = 0;

  this->UpdateProps();

  if (this->ZAxisVisibility)
  {
    renderedSomething += this->ZAxisShaft->RenderOpaqueGeometry(vp);
    renderedSomething += this->ZAxisTip->RenderOpaqueGeometry(vp);
  }
  if (this->XYPlaneVisibility)
  {
    renderedSomething += this->XYPlaneDisk->RenderOpaqueGeometry(vp);
    renderedSomething += this->YAxisShaft->RenderOpaqueGeometry(vp);
    renderedSomething += this->YAxisTip->RenderOpaqueGeometry(vp);
    renderedSomething += this->XAxisShaft->RenderOpaqueGeometry(vp);
    renderedSomething += this->XAxisTip->RenderOpaqueGeometry(vp);
  }

  renderedSomething = (renderedSomething > 0) ? (1) : (0);
  return renderedSomething;
}

//----------------------------------------------------------------------------
int vtkMaptkAxesActor::RenderTranslucentPolygonalGeometry(vtkViewport* vp)
{
  int renderedSomething = 0;

  this->UpdateProps();

  if (this->ZAxisVisibility)
  {
    renderedSomething +=
      this->ZAxisShaft->RenderTranslucentPolygonalGeometry(vp);
    renderedSomething += this->ZAxisTip->RenderTranslucentPolygonalGeometry(vp);
  }
  if (this->XYPlaneVisibility)
  {
    renderedSomething +=
      this->XYPlaneDisk->RenderTranslucentPolygonalGeometry(vp);
    renderedSomething +=
      this->YAxisShaft->RenderTranslucentPolygonalGeometry(vp);
    renderedSomething += this->YAxisTip->RenderTranslucentPolygonalGeometry(vp);
    renderedSomething +=
      this->XAxisShaft->RenderTranslucentPolygonalGeometry(vp);
    renderedSomething += this->XAxisTip->RenderTranslucentPolygonalGeometry(vp);
  }

  renderedSomething = (renderedSomething > 0) ? (1) : (0);
  return renderedSomething;
}
//----------------------------------------------------------------------------
vtkTypeBool vtkMaptkAxesActor::HasTranslucentPolygonalGeometry()
{
  vtkTypeBool result = 0;

  this->UpdateProps();

  if (this->ZAxisVisibility)
  {
    result |= this->ZAxisShaft->HasTranslucentPolygonalGeometry();
    result |= this->ZAxisTip->HasTranslucentPolygonalGeometry();
  }
  if (this->XYPlaneVisibility)
  {
    result |= this->XYPlaneDisk->HasTranslucentPolygonalGeometry();
    result |= this->YAxisShaft->HasTranslucentPolygonalGeometry();
    result |= this->YAxisTip->HasTranslucentPolygonalGeometry();
    result |= this->XAxisShaft->HasTranslucentPolygonalGeometry();
    result |= this->XAxisTip->HasTranslucentPolygonalGeometry();
  }

  return result;
}

//----------------------------------------------------------------------------
int vtkMaptkAxesActor::RenderOverlay(vtkViewport* vp)
{
  int renderedSomething = 0;

  this->UpdateProps();

  if (this->ZAxisVisibility)
  {
    renderedSomething += this->ZAxisShaft->RenderOverlay(vp);
    renderedSomething += this->ZAxisTip->RenderOverlay(vp);
  }
  if (this->XYPlaneVisibility)
  {
    renderedSomething += this->XYPlaneDisk->RenderOverlay(vp);
    renderedSomething += this->YAxisShaft->RenderOverlay(vp);
    renderedSomething += this->YAxisTip->RenderOverlay(vp);
    renderedSomething += this->XAxisShaft->RenderOverlay(vp);
    renderedSomething += this->XAxisTip->RenderOverlay(vp);
  }

  renderedSomething = (renderedSomething > 0) ? (1) : (0);
  return renderedSomething;
}

//----------------------------------------------------------------------------
void vtkMaptkAxesActor::ReleaseGraphicsResources(vtkWindow* win)
{
  this->ZAxisShaft->ReleaseGraphicsResources(win);
  this->ZAxisTip->ReleaseGraphicsResources(win);
  this->YAxisShaft->ReleaseGraphicsResources(win);
  this->YAxisTip->ReleaseGraphicsResources(win);
  this->XAxisShaft->ReleaseGraphicsResources(win);
  this->XAxisTip->ReleaseGraphicsResources(win);
  this->XYPlaneDisk->ReleaseGraphicsResources(win);
}

//----------------------------------------------------------------------------
void vtkMaptkAxesActor::ShallowCopy(vtkProp* prop)
{
  vtkMaptkAxesActor* m = vtkMaptkAxesActor::SafeDownCast(prop);
  if (m)
  {
    this->SetZAxisVisibility(m->GetZAxisVisibility());
    this->SetXYPlaneVisibility(m->GetXYPlaneVisibility());
    this->SetAxesLength(m->GetAxesLength());
  }

  // Forward to superclass
  Superclass::ShallowCopy(prop);
}
