/*ckwg +29
 * Copyright 2016 by Kitware, Inc.
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

#include "vtkMaptkCamera.h"

#include <vital/io/camera_io.h>

#include <vtkObjectFactory.h>
#include <vtkMath.h>
#include <vtkMatrix4x4.h>

#include <vtksys/SystemTools.hxx>

vtkStandardNewMacro(vtkMaptkCamera);

namespace // anonymous
{

//-----------------------------------------------------------------------------
void BuildCamera(vtkMaptkCamera* out, kwiver::vital::camera_sptr const& in,
                 kwiver::vital::camera_intrinsics_sptr const& ci)
{
  // Get camera parameters
  auto const pixelAspect = ci->aspect_ratio();
  auto const focalLength = ci->focal_length();

  int imageWidth, imageHeight;
  out->GetImageDimensions(imageWidth, imageHeight);

  double aspectRatio = pixelAspect * imageWidth / imageHeight;
  out->SetAspectRatio(aspectRatio);

  double fov =
    vtkMath::DegreesFromRadians(2.0 * atan(0.5 * imageHeight / focalLength));
  out->SetViewAngle(fov);

  // Compute camera vectors from matrix
  auto const& rotationMatrix = in->rotation().quaternion().toRotationMatrix();

  auto up = -rotationMatrix.row(1).transpose();
  auto view = rotationMatrix.row(2).transpose();
  auto center = in->center();

  out->SetPosition(center[0], center[1], center[2]);
  out->SetViewUp(up[0], up[1], up[2]);

  auto const& focus = center + (view * out->GetDistance() / view.norm());
  out->SetFocalPoint(focus[0], focus[1], focus[2]);
}

} // namespace <anonymous>

//-----------------------------------------------------------------------------
vtkMaptkCamera::vtkMaptkCamera()
{
  this->ImageDimensions[0] = this->ImageDimensions[1] = -1;
}

//-----------------------------------------------------------------------------
vtkMaptkCamera::~vtkMaptkCamera()
{
}

//-----------------------------------------------------------------------------
kwiver::vital::camera_sptr vtkMaptkCamera::GetCamera() const
{
  return this->MaptkCamera;
}

//-----------------------------------------------------------------------------
void vtkMaptkCamera::SetCamera(kwiver::vital::camera_sptr const& camera)
{
  this->MaptkCamera = camera;
}

//-----------------------------------------------------------------------------
bool vtkMaptkCamera::ProjectPoint(kwiver::vital::vector_3d const& in,
                                  double (&out)[2])
{
  if (this->MaptkCamera->depth(in) < 0.0)
  {
    return false;
  }

  auto const& ppos = this->MaptkCamera->project(in);
  out[0] = ppos[0];
  out[1] = ppos[1];
  return true;
}

//-----------------------------------------------------------------------------
bool vtkMaptkCamera::Update()
{
  auto const& ci = this->MaptkCamera->intrinsics();

  if (this->ImageDimensions[0] == -1 || this->ImageDimensions[1] == -1)
  {
    // Guess image size
    auto const& s = ci->principal_point() * 2.0;
    this->ImageDimensions[0] = s[0];
    this->ImageDimensions[1] = s[1];
  }

  BuildCamera(this, this->MaptkCamera, ci);

  // here for now, but this is something we actually want to be a property
  // of the representation... that is, the depth (size) displayed for the camera
  // as determined by the far clipping plane
  auto const depth = 15.0;
  this->SetClippingRange(0.01, depth);
  return true;
}

//-----------------------------------------------------------------------------
void vtkMaptkCamera::GetFrustumPlanes(double planes[24])
{
  // Need to add timing (modfied time) logic to determine if need to Update()
  this->Superclass::GetFrustumPlanes(this->AspectRatio, planes);
}

//-----------------------------------------------------------------------------
void vtkMaptkCamera::GetTransform(vtkMatrix4x4* out, double const plane[4])
{
  // Build camera matrix
  auto const k = this->MaptkCamera->intrinsics()->as_matrix();
  auto const t = this->MaptkCamera->translation();
  auto const r = kwiver::vital::matrix_3x3d(this->MaptkCamera->rotation());

  auto const kr = kwiver::vital::matrix_3x3d(k * r);
  auto const kt = kwiver::vital::vector_3d(k * t);

  out->SetElement(0, 0, kr(0, 0));
  out->SetElement(0, 1, kr(0, 1));
  out->SetElement(0, 3, kr(0, 2));
  out->SetElement(1, 0, kr(1, 0));
  out->SetElement(1, 1, kr(1, 1));
  out->SetElement(1, 3, kr(1, 2));
  out->SetElement(3, 0, kr(2, 0));
  out->SetElement(3, 1, kr(2, 1));
  out->SetElement(3, 3, kr(2, 2));

  out->SetElement(0, 3, kt[0]);
  out->SetElement(1, 3, kt[1]);
  out->SetElement(3, 3, kt[2]);

  // Insert plane coefficients into matrix to build plane-to-image projection
  out->SetElement(2, 0, plane[0]);
  out->SetElement(2, 1, plane[1]);
  out->SetElement(2, 2, plane[2]);
  out->SetElement(2, 3, plane[3]);

  // Invert to get image-to-plane projection
  out->Invert();
}

//-----------------------------------------------------------------------------
void vtkMaptkCamera::DeepCopy(vtkMaptkCamera* source)
{
  this->Superclass::DeepCopy(source);

  this->ImageDimensions[0] = source->ImageDimensions[0];
  this->ImageDimensions[1] = source->ImageDimensions[1];
  this->AspectRatio = source->AspectRatio;
  this->MaptkCamera = source->MaptkCamera;
}

//-----------------------------------------------------------------------------
void vtkMaptkCamera::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}
