// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

// maptk includes
#include "vtkMaptkPointPicker.h"

// VTK includes
#include <vtkCommand.h>
#include <vtkNew.h>
#include <vtkObjectFactory.h>
#include <vtkRenderer.h>

vtkStandardNewMacro(vtkMaptkPointPicker);

//----------------------------------------------------------------------------
int vtkMaptkPointPicker::Pick3DPoint(double selectionPt[3],
                                     double focalPt[3],
                                     vtkRenderer* ren)
{
  // Initialize the picking process
  this->Initialize();
  this->Renderer = ren;

  // Invoke start pick method if defined
  this->InvokeEvent(vtkCommand::StartPickEvent, nullptr);

  int result = this->Pick3DInternal(ren, selectionPt, focalPt);

  // Invoke end pick method if defined
  this->InvokeEvent(vtkCommand::EndPickEvent, nullptr);

  return result;
}

//----------------------------------------------------------------------------
void vtkMaptkPointPicker::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent.GetNextIndent());
}
