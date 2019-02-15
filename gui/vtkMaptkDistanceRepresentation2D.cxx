/*ckwg +29
 * Copyright 2019 by Kitware, Inc.
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
#include "vtkMaptkDistanceRepresentation2D.h"

// VTK includes
#include <vtkAxisActor2D.h>
#include <vtkHandleRepresentation.h>
#include <vtkObjectFactory.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkTextProperty.h>

vtkStandardNewMacro(vtkMaptkDistanceRepresentation2D);

//----------------------------------------------------------------------------
void vtkMaptkDistanceRepresentation2D::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}

//----------------------------------------------------------------------------
void vtkMaptkDistanceRepresentation2D::BuildRepresentation()
{
  if (this->GetMTime() > this->BuildTime ||
      this->AxisActor->GetMTime() > this->BuildTime ||
      this->AxisActor->GetTitleTextProperty()->GetMTime() > this->BuildTime ||
      this->Point1Representation->GetMTime() > this->BuildTime ||
      this->Point2Representation->GetMTime() > this->BuildTime ||
      (this->Renderer && this->Renderer->GetVTKWindow() &&
       this->Renderer->GetVTKWindow()->GetMTime() > this->BuildTime))
  {
    double oldDist = this->Distance;
    this->Superclass::BuildRepresentation();
    if (!this->GetComputeDistance())
    {
      this->Distance = oldDist;

      char string[512];
      snprintf(string,
               sizeof(string),
               this->LabelFormat,
               this->Distance * this->Scale);
      this->AxisActor->SetTitle(string);
      this->BuildTime.Modified();
    }
  }
}
