// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

// maptk includes
#include "vtkMaptkPointPlacer.h"

// VTK includes
#include <vtkCamera.h>
#include <vtkCellPicker.h>
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

  double position[4] = { 0.0, 0.0, 0.0, 0.0 };
  // Compute tolerance for the pickers based on the render window size
  int height = 0, width = 0;
  ren->GetTiledSize(&height, &width);
  double tolerance = 3.0 / sqrt(height * height + width * width);

  // Set up pickers
  vtkNew<vtkPointPicker> pointPicker;
  pointPicker->SetTolerance(tolerance);
  vtkNew<vtkCellPicker> cellPicker;
  cellPicker->SetTolerance(tolerance);

  int pointPicked = pointPicker->Pick(displayPos[0], displayPos[1], 0, ren);
  int cellPicked = cellPicker->Pick(displayPos[0], displayPos[1], 0, ren);
  if (pointPicked || cellPicked)
  {
    vtkPicker* picker = nullptr;
    if (pointPicked && cellPicked)
    {
      // Compute closest picked position
      double* camPos = ren->GetActiveCamera()->GetPosition();
      double* pPos = pointPicker->GetPickPosition();
      double* cPos = cellPicker->GetPickPosition();
      if (vtkMath::Distance2BetweenPoints(camPos, pPos) >
          vtkMath::Distance2BetweenPoints(camPos, cPos))
      {
        picker = cellPicker;
      }
      else
      {
        picker = pointPicker;
      }
    }
    else if (cellPicked)
    {
      picker = cellPicker;
    }
    else
    {
      picker = pointPicker;
    }
    double pickedPos[3];
    double focalPoint[4];
    picker->GetPickPosition(pickedPos);
    vtkInteractorObserver::ComputeWorldToDisplay(
      ren, pickedPos[0], pickedPos[1], pickedPos[2], focalPoint);
    vtkInteractorObserver::ComputeDisplayToWorld(
      ren, displayPos[0], displayPos[1], focalPoint[2], position);
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

    if (!vtkPlane::IntersectWithLine(
          nearWorldPoint, farWorldPoint, normal, origin, distance, position))
    {
      ren->SetDisplayPoint(displayPos[0], displayPos[1], 0.5);
      ren->DisplayToWorld();
      ren->GetWorldPoint(position);
    }
  }
  worldPos[0] = position[0];
  worldPos[1] = position[1];
  worldPos[2] = position[2];
  return 1;
}

//----------------------------------------------------------------------------
void vtkMaptkPointPlacer::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}
