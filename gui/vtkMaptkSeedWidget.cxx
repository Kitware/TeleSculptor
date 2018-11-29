/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkMaptkSeedWidget.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "vtkMaptkSeedWidget.h"

// VTK includes
#include <vtkCallbackCommand.h>
#include <vtkHandleRepresentation.h>
#include <vtkHandleWidget.h>
#include <vtkObjectFactory.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSeedRepresentation.h>
#include <vtkWidgetCallbackMapper.h>
#include <vtkWidgetEvent.h>

// STL includes
#include <iterator>
#include <list>

vtkStandardNewMacro(vtkMaptkSeedWidget);

// The vtkSeedList is a PIMPLed list<T>.
class vtkSeedList : public std::list<vtkHandleWidget*>
{
};
typedef std::list<vtkHandleWidget*>::iterator vtkSeedListIterator;

//----------------------------------------------------------------------
vtkMaptkSeedWidget::vtkMaptkSeedWidget()
{
  this->ManagesCursor = 1;

  // These are the event callbacks supported by this widget
  this->CallbackMapper->SetCallbackMethod(vtkCommand::LeftButtonPressEvent,
                                          vtkWidgetEvent::AddPoint,
                                          this,
                                          vtkMaptkSeedWidget::AddPointAction);
  this->CallbackMapper->SetCallbackMethod(vtkCommand::MouseMoveEvent,
                                          vtkWidgetEvent::Move,
                                          this,
                                          vtkMaptkSeedWidget::MoveAction);
  this->CallbackMapper->SetCallbackMethod(vtkCommand::LeftButtonReleaseEvent,
                                          vtkWidgetEvent::EndSelect,
                                          this,
                                          vtkMaptkSeedWidget::EndSelectAction);
}

//----------------------------------------------------------------------
vtkMaptkSeedWidget::~vtkMaptkSeedWidget()
{
}

//----------------------------------------------------------------------
void vtkMaptkSeedWidget::DeleteSeed(int i)
{
  this->Superclass::DeleteSeed(i);
  this->HighlightActiveSeed();
}

//----------------------------------------------------------------------
void vtkMaptkSeedWidget::SetEnabled(int enabling)
{
  this->Superclass::SetEnabled(enabling);
  this->HighlightActiveSeed();
}

//-------------------------------------------------------------------------
void vtkMaptkSeedWidget::AddPointAction(vtkAbstractWidget* w)
{
  vtkMaptkSeedWidget* self = reinterpret_cast<vtkMaptkSeedWidget*>(w);

  // Need to distinguish between placing handles and manipulating handles
  if (self->WidgetState == vtkMaptkSeedWidget::MovingSeed)
  {
    // self->HighlightActiveSeed();
    return;
  }

  // compute some info we need for all cases
  int X = self->Interactor->GetEventPosition()[0];
  int Y = self->Interactor->GetEventPosition()[1];

  // When a seed is placed, a new handle widget must be created and enabled.
  int state = self->WidgetRep->ComputeInteractionState(X, Y);
  if (state == vtkSeedRepresentation::NearSeed)
  {
    self->WidgetState = vtkMaptkSeedWidget::MovingSeed;

    // Invoke an event on ourself for the handles
    self->InvokeEvent(vtkCommand::LeftButtonPressEvent, nullptr);
    self->Superclass::StartInteraction();
    vtkSeedRepresentation* rep =
      static_cast<vtkSeedRepresentation*>(self->WidgetRep);
    int seedIdx = rep->GetActiveHandle();
    self->InvokeEvent(vtkCommand::StartInteractionEvent, &seedIdx);

    self->HighlightActiveSeed();
    self->EventCallbackCommand->SetAbortFlag(1);
  }
  else if (self->WidgetState != vtkMaptkSeedWidget::PlacedSeeds)
  {
    if (!self->Interactor->GetControlKey())
    {
      return;
    }

    // we are placing a new seed. Just make sure we aren't in a mode which
    // dictates we've placed all seeds.

    self->WidgetState = vtkMaptkSeedWidget::PlacingSeeds;
    double e[3];
    e[2] = 0.0;
    e[0] = static_cast<double>(X);
    e[1] = static_cast<double>(Y);

    vtkSeedRepresentation* rep =
      reinterpret_cast<vtkSeedRepresentation*>(self->WidgetRep);
    // if the handle representation is constrained, check to see if
    // the position follows the constraint.
    if (!rep->GetHandleRepresentation()->CheckConstraint(
          self->GetCurrentRenderer(), e))
    {
      return;
    }
    int currentHandleNumber = rep->CreateHandle(e);
    vtkHandleWidget* currentHandle = self->CreateNewHandle();
    rep->SetSeedDisplayPosition(currentHandleNumber, e);
    // Now that the seed is placed, reset the point placer to ensure free
    // motion of the handle
    rep->GetHandleRepresentation(currentHandleNumber)->SetPointPlacer(nullptr);
    currentHandle->SetEnabled(1);
    self->InvokeEvent(vtkCommand::PlacePointEvent, &(currentHandleNumber));
    self->InvokeEvent(vtkCommand::InteractionEvent, &(currentHandleNumber));

    self->HighlightActiveSeed();
    self->EventCallbackCommand->SetAbortFlag(1);
  }
}

//-------------------------------------------------------------------------
void vtkMaptkSeedWidget::MoveAction(vtkAbstractWidget* w)
{
  vtkMaptkSeedWidget* self = reinterpret_cast<vtkMaptkSeedWidget*>(w);

  self->InvokeEvent(vtkCommand::MouseMoveEvent, nullptr);

  // set the cursor shape to a hand if we are near a seed.
  int X = self->Interactor->GetEventPosition()[0];
  int Y = self->Interactor->GetEventPosition()[1];
  int state = self->WidgetRep->ComputeInteractionState(X, Y);

  // Change the cursor shape to a hand and invoke an interaction event if we
  // are near the seed
  if (state == vtkSeedRepresentation::NearSeed)
  {
    int cShape = VTK_CURSOR_HAND;
    self->RequestCursorShape(cShape);
    self->InvokeEvent(vtkCommand::CursorChangedEvent, &cShape);

    vtkSeedRepresentation* rep =
      static_cast<vtkSeedRepresentation*>(self->WidgetRep);
    int seedIdx = rep->GetActiveHandle();
    self->InvokeEvent(vtkCommand::InteractionEvent, &seedIdx);

    self->EventCallbackCommand->SetAbortFlag(1);
  }
  else
  {
    int cShape = VTK_CURSOR_DEFAULT;
    self->RequestCursorShape(cShape);
    self->InvokeEvent(vtkCommand::CursorChangedEvent, &cShape);
  }

  self->Render();
}

//-------------------------------------------------------------------------
void vtkMaptkSeedWidget::EndSelectAction(vtkAbstractWidget* w)
{
  vtkMaptkSeedWidget* self = reinterpret_cast<vtkMaptkSeedWidget*>(w);

  // Do nothing if outside
  if (self->WidgetState != vtkMaptkSeedWidget::MovingSeed)
  {
    return;
  }

  // Revert back to the mode we were in prior to selection.
  self->WidgetState = self->Defining ? vtkMaptkSeedWidget::PlacingSeeds
                                     : vtkMaptkSeedWidget::PlacedSeeds;

  // Invoke event for seed handle
  self->InvokeEvent(vtkCommand::LeftButtonReleaseEvent, nullptr);
  self->EventCallbackCommand->SetAbortFlag(1);
  self->InvokeEvent(vtkCommand::EndInteractionEvent, nullptr);
  self->Superclass::EndInteraction();
  self->HighlightActiveSeed();
}

//----------------------------------------------------------------------
vtkHandleWidget* vtkMaptkSeedWidget::CreateNewHandle()
{
  vtkHandleWidget* w = this->Superclass::CreateNewHandle();
  w->SetShowInactive(true);

  return w;
}

//----------------------------------------------------------------------
void vtkMaptkSeedWidget::HighlightActiveSeed()
{
  vtkSeedListIterator iter = this->Seeds->begin();
  vtkSeedRepresentation* rep =
    reinterpret_cast<vtkSeedRepresentation*>(this->WidgetRep);
  int activeHandle = rep->GetActiveHandle();
  if (activeHandle < 0 || activeHandle >= rep->GetNumberOfSeeds())
  {
    activeHandle = rep->GetNumberOfSeeds() - 1;
  }
  for (; iter != this->Seeds->end(); ++iter)
  {
    if (this->GetEnabled() &&
        (std::distance(this->Seeds->begin(), iter) == activeHandle))
    {
      (*iter)->GetHandleRepresentation()->Highlight(1);
      this->InvokeEvent(vtkMaptkSeedWidget::ActiveSeedChangedEvent,
                        &activeHandle);
    }
    else
    {
      (*iter)->GetHandleRepresentation()->Highlight(0);
    }
  }
  this->Render();
}

//----------------------------------------------------------------------
void vtkMaptkSeedWidget::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}
