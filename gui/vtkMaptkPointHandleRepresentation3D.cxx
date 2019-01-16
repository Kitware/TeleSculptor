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

// MAPTK includes
#include "vtkMaptkPointHandleRepresentation3D.h"

// VTK includes
#include <vtkCellPicker.h>
#include <vtkObjectFactory.h>
#include <vtkRenderer.h>

vtkStandardNewMacro(vtkMaptkPointHandleRepresentation3D);

//----------------------------------------------------------------------------
void vtkMaptkPointHandleRepresentation3D::PrintSelf(ostream& os,
                                                    vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}

// Override to ensure that the pick tolerance is always about the same as
// handle size.
//----------------------------------------------------------------------------
int vtkMaptkPointHandleRepresentation3D::ComputeInteractionState(
  int X,
  int Y,
  int vtkNotUsed(modify))
{
  this->VisibilityOn(); // actor must be on to be picked

  vtkAssemblyPath* path = this->GetAssemblyPath(X, Y, 0., this->CursorPicker);

  double focus[3];
  this->Cursor3D->GetFocalPoint(focus);
  double d[3];
  this->GetDisplayPosition(d);

  if (path != nullptr && (std::abs(d[0] - X) < this->GetHandleSize() / 2.0) &&
      (std::abs(d[1] - Y) < this->GetHandleSize() / 2.0))
  {
    this->InteractionState = vtkHandleRepresentation::Nearby;
  }
  else
  {
    this->InteractionState = vtkHandleRepresentation::Outside;
    if (this->ActiveRepresentation)
    {
      this->VisibilityOff();
    }
  }

  return this->InteractionState;
}
