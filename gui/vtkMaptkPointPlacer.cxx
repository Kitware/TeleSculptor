/*ckwg +29
 * Copyright 2018 by Kitware, Inc.
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

// maptk includes
#include "vtkMaptkPointPlacer.h"

// VTK includes
#include <vtkInteractorObserver.h>
#include <vtkNew.h>
#include <vtkObjectFactory.h>
#include <vtkPlane.h>
#include <vtkPointPicker.h>
#include <vtkRenderer.h>

vtkStandardNewMacro(vtkMaptkPointPlacer);

//----------------------------------------------------------------------------
int vtkMaptkPointPlacer::ComputeWorldPosition(vtkRenderer* ren,
                                              double displayPos[2],
                                              double worldPos[3],
                                              double vtkNotUsed(worldOrient)[9])
{
  if (!ren)
  {
    return 0;
  }

  int valid = 0;
  double position[3];
  vtkNew<vtkPointPicker> pointPicker;
  // Compute tolerance for the point picker based on the render window size
  int height = 0, width = 0;
  ren->GetTiledSize(&height, &width);
  double tolerance = 1.0 / (height + width);
  pointPicker->SetTolerance(tolerance);
  if (pointPicker->Pick(displayPos[0], displayPos[1], 0, ren))
  {
    double pickedPos[3];
    double focalPoint[4];
    pointPicker->GetPickPosition(pickedPos);
    vtkInteractorObserver::ComputeWorldToDisplay(
      ren, pickedPos[0], pickedPos[1], pickedPos[2], focalPoint);
    vtkInteractorObserver::ComputeDisplayToWorld(
      ren, displayPos[0], displayPos[1], focalPoint[2], position);
    valid = 1;
  }
  else
  {
    // If the picker failed to pick a point, place the point on the z=0 plane.
    double nearWorldPoint[4];
    double farWorldPoint[4];
    double tmp[3];

    tmp[0] = displayPos[0];
    tmp[1] = displayPos[1];
    tmp[2] = 0.0; // near plane

    ren->SetDisplayPoint(tmp);
    ren->DisplayToWorld();
    ren->GetWorldPoint(nearWorldPoint);

    tmp[2] = 1.0; // far plane
    ren->SetDisplayPoint(tmp);
    ren->DisplayToWorld();
    ren->GetWorldPoint(farWorldPoint);

    double normal[3] = { 0, 0, 1 };
    double origin[3] = { 0, 0, 0 };
    double distance;

    if (vtkPlane::IntersectWithLine(
          nearWorldPoint, farWorldPoint, normal, origin, distance, position))
    {
      valid = 1;
    }
  }
  if (valid)
  {
    worldPos[0] = position[0];
    worldPos[1] = position[1];
    worldPos[2] = position[2];
    return 1;
  }
  return 0;
}

//----------------------------------------------------------------------------
void vtkMaptkPointPlacer::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}
