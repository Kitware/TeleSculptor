// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

// TeleSculptor includes
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
vtkMaptkDistanceRepresentation2D::vtkMaptkDistanceRepresentation2D()
{
  this->RulerMode = true;
  this->AxisActor->SetTickLength(10);
  this->AxisActor->SetNumberOfMinorTicks(10);
  this->SetRulerDistance(1.0);
}

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
