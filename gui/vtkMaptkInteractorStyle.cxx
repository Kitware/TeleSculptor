// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

// maptk includes
#include "vtkMaptkInteractorStyle.h"

#include "Utils.h"

// VTK includes
#include <vtkCamera.h>
#include <vtkNew.h>
#include <vtkObjectFactory.h>
#include <vtkPointPicker.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkRendererCollection.h>
#include <vtkTransform.h>

vtkStandardNewMacro(vtkMaptkInteractorStyle);

//----------------------------------------------------------------------------
vtkMaptkInteractorStyle::vtkMaptkInteractorStyle()
{
  this->AddObserver(
    vtkCommand::TimerEvent, this, &vtkMaptkInteractorStyle::TimerCallback);
}

//----------------------------------------------------------------------------
void vtkMaptkInteractorStyle::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}

//----------------------------------------------------------------------------
void vtkMaptkInteractorStyle::TimerCallback(vtkObject* vtkNotUsed(caller),
                                            unsigned long vtkNotUsed(eventId),
                                            void* callData)
{
  int timerId = (callData ? *(reinterpret_cast<int*>(callData)) : 1);
  if (timerId != this->TimerId)
  {
    // This is a timer event for another timer that we are not interested in.
    return;
  }
  // If the time is up, destroy the timer and reset timer Id.
  this->DestroyTimer();
}

//----------------------------------------------------------------------------
void vtkMaptkInteractorStyle::DestroyTimer()
{
  this->Interactor->DestroyTimer(this->TimerId);
  this->TimerId = -1;
  this->TimerStatus = TimedOut;
}

//----------------------------------------------------------------------------
void vtkMaptkInteractorStyle::OnLeftButtonDown()
{
  int* eventPos = this->Interactor->GetEventPosition();

  if (this->TimerStatus == TimedOut)
  {
    // This is the first click. Create timer.
    this->TimerId =
      this->Interactor->CreateOneShotTimer(GetDoubleClickInterval());
    this->TimerStatus = Timing;
  }
  else
  {
    // Still timing, i.e. double clicked
    this->DestroyTimer();
    vtkRenderer* ren =
      this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer();
    if (ren)
    {
      double worldPt[3];
      vtkNew<vtkPointPicker> picker;
      picker->SetTolerance(0.01);
      int picked = picker->Pick(eventPos[0], eventPos[1], 0, ren);
      picker->GetPickPosition(worldPt);
      if (!picked)
      {
        ren->SetDisplayPoint(eventPos[0], eventPos[1], 0);
        ren->DisplayToWorld();
        double* wPt = ren->GetWorldPoint();
        worldPt[0] = wPt[0];
        worldPt[1] = wPt[1];
        worldPt[2] = wPt[2];
      }

      vtkCamera* cam = ren->GetActiveCamera();
      cam->SetFocalPoint(worldPt);

      cam->OrthogonalizeViewUp();
      ren->ResetCameraClippingRange();
    }
  }

  // Forward events
  this->Superclass::OnLeftButtonDown();
}
