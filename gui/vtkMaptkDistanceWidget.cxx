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
          if (strcmp(iren->GetKeySym(), "z") == 0)
          {
            this->DistanceWidget->SetConstraintMode(2);
          }
          else if ((strcmp(iren->GetKeySym(), "x") == 0) ||
                   (strcmp(iren->GetKeySym(), "y") == 0))
          {
            this->DistanceWidget->SetConstraintMode(0);
          }
          else if (iren->GetKeyCode() == 27)
          {
            // Escape key
            this->DistanceWidget->SetConstraintMode(-1);
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
  if (repr)
  {
    repr->SetConstrained((this->ConstraintMode < 0 ? false : true));
    repr->SetConstraintAxis(this->ConstraintMode);
  }

  // reset the other representation since we're not interacting with it
  repr = vtkMaptkPointHandleRepresentation3D::SafeDownCast(
    otherW->GetHandleRepresentation());
  if (repr)
  {
    repr->SetConstrained(false);
  }
}

//----------------------------------------------------------------------
void vtkMaptkDistanceWidget::DistanceInteraction(int id)
{
  this->InvokeEvent(this->DistanceInteractionEvent, &id);
}
