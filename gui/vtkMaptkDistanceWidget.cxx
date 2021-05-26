// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

// TeleSculptor includes
#include "vtkMaptkDistanceWidget.h"

#include "vtkMaptkPointHandleRepresentation3D.h"

// VTK includes
#include <vtkCommand.h>
#include <vtkHandleWidget.h>
#include <vtkObjectFactory.h>
#include <vtkRenderWindowInteractor.h>

vtkStandardNewMacro(vtkMaptkDistanceWidget);

//----------------------------------------------------------------------------
// The distance widget observes its two handles.
// Here we create the command/observer classes to respond to the
// handle widgets.
class vtkMaptkDistanceWidgetCallback : public vtkCommand
{
public:
  static vtkMaptkDistanceWidgetCallback* New()
  {
    return new vtkMaptkDistanceWidgetCallback;
  }
  void Execute(vtkObject* caller, unsigned long eventId, void*) override
  {
    switch (eventId)
    {
      case vtkCommand::InteractionEvent:
      {
        this->DistanceWidget->UpdateRepresentationConstraint(
          this->HandleNumber);
        this->DistanceWidget->DistanceInteraction(this->HandleNumber);
        break;
      }
      case vtkCommand::KeyPressEvent:
      {
        vtkRenderWindowInteractor* iren =
          dynamic_cast<vtkRenderWindowInteractor*>(caller);
        if (iren)
        {
          if (this->DistanceWidget->ConstraintMode >= 0)
          {
            if (iren->GetKeyCode() == 27)
            {
              // Escape key
              this->DistanceWidget->SetConstraintMode(-1);
            }
          }
          else
          {
            if (strcmp(iren->GetKeySym(), "z") == 0)
            {
              this->DistanceWidget->SetConstraintMode(2);
            }
            else if ((strcmp(iren->GetKeySym(), "x") == 0) ||
                     (strcmp(iren->GetKeySym(), "y") == 0))
            {
              this->DistanceWidget->SetConstraintMode(0);
            }
          }
        }
        break;
      }
      case vtkCommand::CursorChangedEvent:
      {
        this->DistanceWidget->UpdateRepresentationConstraint(
          this->HandleNumber);
        break;
      }
      case vtkCommand::KeyReleaseEvent:
      {
        vtkRenderWindowInteractor* iren =
          dynamic_cast<vtkRenderWindowInteractor*>(caller);
        if (iren)
        {
          int mode = this->DistanceWidget->ConstraintMode;
          if (((strcmp(iren->GetKeySym(), "z") == 0) && mode == 2) ||
              ((strcmp(iren->GetKeySym(), "x") == 0 ||
                (strcmp(iren->GetKeySym(), "y") == 0)) &&
               mode == 0))
          {

            this->DistanceWidget->SetConstraintMode(-1);
          }
        }
        break;
      }
    }
  }
  int HandleNumber;
  vtkMaptkDistanceWidget* DistanceWidget;
};

//----------------------------------------------------------------------------
vtkMaptkDistanceWidget::vtkMaptkDistanceWidget()
{
  this->DistanceInteractionEvent = vtkCommand::UserEvent + 100;
  this->MaptkDistanceWidgetCallback1 = vtkMaptkDistanceWidgetCallback::New();
  this->MaptkDistanceWidgetCallback1->HandleNumber = 0;
  this->MaptkDistanceWidgetCallback1->DistanceWidget = this;
  this->Point1Widget->EnableAxisConstraintOff();
  this->Point1Widget->AddObserver(vtkCommand::InteractionEvent,
                                  this->MaptkDistanceWidgetCallback1,
                                  this->Priority);
  this->Point1Widget->AddObserver(vtkCommand::CursorChangedEvent,
                                  this->MaptkDistanceWidgetCallback1,
                                  this->Priority);

  this->MaptkDistanceWidgetCallback2 = vtkMaptkDistanceWidgetCallback::New();
  this->MaptkDistanceWidgetCallback2->HandleNumber = 1;
  this->MaptkDistanceWidgetCallback2->DistanceWidget = this;
  this->Point2Widget->EnableAxisConstraintOff();
  this->Point2Widget->AddObserver(vtkCommand::InteractionEvent,
                                  this->MaptkDistanceWidgetCallback2,
                                  this->Priority);
  this->Point2Widget->AddObserver(vtkCommand::CursorChangedEvent,
                                  this->MaptkDistanceWidgetCallback2,
                                  this->Priority);
}

//----------------------------------------------------------------------------
vtkMaptkDistanceWidget::~vtkMaptkDistanceWidget()
{
  if (this->Interactor)
  {
    this->Interactor->RemoveObserver(this->MaptkDistanceWidgetCallback1);
  }
  this->Point1Widget->RemoveObserver(this->MaptkDistanceWidgetCallback1);
  this->MaptkDistanceWidgetCallback1->Delete();
  this->Point2Widget->RemoveObserver(this->MaptkDistanceWidgetCallback2);
  this->MaptkDistanceWidgetCallback2->Delete();
}

//----------------------------------------------------------------------------
void vtkMaptkDistanceWidget::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}

//----------------------------------------------------------------------------
void vtkMaptkDistanceWidget::SetInteractor(vtkRenderWindowInteractor* iren)
{
  Superclass::SetInteractor(iren);
  if (iren)
  {
    iren->AddObserver(vtkCommand::KeyPressEvent,
                      this->MaptkDistanceWidgetCallback1,
                      this->Priority);
    iren->AddObserver(vtkCommand::KeyReleaseEvent,
                      this->MaptkDistanceWidgetCallback1,
                      this->Priority);
  }
}

//----------------------------------------------------------------------------
void vtkMaptkDistanceWidget::SetConstraintMode(int mode)
{
  if (this->ConstraintMode == mode)
  {
    return;
  }
  if (mode < 0 || mode > 2)
  {
    this->ConstraintMode = -1;
  }
  else
  {
    this->ConstraintMode = mode;
  }
}

// These are callbacks that are active when the user is manipulating the
// handles of the measure widget.
//----------------------------------------------------------------------
void vtkMaptkDistanceWidget::UpdateRepresentationConstraint(int id)
{
  vtkHandleWidget* w = nullptr;
  vtkHandleWidget* otherW = nullptr;
  switch (id)
  {
    case 0:
    {
      w = this->Point1Widget;
      otherW = this->Point2Widget;
      break;
    }
    case 1:
    {
      w = this->Point2Widget;
      otherW = this->Point1Widget;
      break;
    }
  }

  vtkMaptkPointHandleRepresentation3D* repr =
    vtkMaptkPointHandleRepresentation3D::SafeDownCast(
      w->GetHandleRepresentation());
  vtkMaptkPointHandleRepresentation3D* otherRepr =
    vtkMaptkPointHandleRepresentation3D::SafeDownCast(
      otherW->GetHandleRepresentation());
  if (repr)
  {
    repr->SetConstrained(this->ConstraintMode >= 0);
    repr->SetConstraintAxis(this->ConstraintMode);
    if (repr->GetConstrained() && otherRepr)
    {
      double pt[3], newPt[3];
      repr->GetWorldPosition(newPt);
      otherRepr->GetWorldPosition(pt);
      switch (this->ConstraintMode)
      {
        case 0:
        case 1:
        {
          newPt[2] = pt[2];
          break;
        }
        case 2:
        {
          newPt[0] = pt[0];
          newPt[1] = pt[1];
          break;
        }
      }
      repr->SetWorldPosition(newPt);
    }
  }

  // reset the other representation since we're not interacting with it
  if (otherRepr)
  {
    otherRepr->SetConstrained(false);
  }
}

//----------------------------------------------------------------------
void vtkMaptkDistanceWidget::DistanceInteraction(int id)
{
  this->InvokeEvent(this->DistanceInteractionEvent, &id);
}
