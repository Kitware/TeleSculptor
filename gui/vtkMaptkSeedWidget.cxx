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
#include <vtkEvent.h>
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
  this->CallbackMapper->SetCallbackMethod(vtkCommand::LeftButtonReleaseEvent,
                                          vtkWidgetEvent::EndSelect,
                                          this,
                                          vtkMaptkSeedWidget::EndSelectAction);
  this->CallbackMapper->SetCallbackMethod(vtkCommand::KeyPressEvent,
                                          vtkEvent::NoModifier,
                                          127,
                                          1,
                                          "Delete",
                                          vtkWidgetEvent::Delete,
                                          this,
                                          vtkMaptkSeedWidget::DeleteAction);
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
int vtkMaptkSeedWidget::FindSeed(vtkHandleWidget* handle)
{
  auto iter = this->Seeds->begin();
  auto const end = this->Seeds->end();

  for (int n = 0; iter != end; ++n, ++iter)
  {
    if (*iter == handle)
    {
      return n;
    }
  }

  return -1;
}

//----------------------------------------------------------------------
void vtkMaptkSeedWidget::SetEnabled(int enabling)
{
  this->Superclass::SetEnabled(enabling);
  this->WidgetState =
    enabling ? vtkMaptkSeedWidget::PlacingSeeds : vtkMaptkSeedWidget::Start;
  this->HighlightActiveSeed();
}

//-------------------------------------------------------------------------
void vtkMaptkSeedWidget::AddPointAction(vtkAbstractWidget* w)
{
  vtkMaptkSeedWidget* self = reinterpret_cast<vtkMaptkSeedWidget*>(w);

  // Need to distinguish between placing handles and manipulating handles
  if (self->WidgetState == vtkMaptkSeedWidget::MovingSeed)
  {
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
// Implemented here to prevent the "RemoveHandle" call made by
// vtkSeedWidget::DeleteAction
void vtkMaptkSeedWidget::DeleteAction(vtkAbstractWidget* w)
{
  vtkMaptkSeedWidget* self = reinterpret_cast<vtkMaptkSeedWidget*>(w);

  // Do nothing if outside
  if (self->WidgetState != vtkSeedWidget::PlacingSeeds)
  {
    return;
  }

  // Remove last seed
  vtkSeedRepresentation* rep =
    reinterpret_cast<vtkSeedRepresentation*>(self->WidgetRep);
  int removeId = rep->GetActiveHandle();
  removeId =
    removeId != -1 ? removeId : static_cast<int>(self->Seeds->size()) - 1;
  // Invoke event for seed handle before actually deleting
  self->InvokeEvent(vtkCommand::DeletePointEvent, &(removeId));

  self->DeleteSeed(removeId);
  // Got this event, abort processing if it
  self->EventCallbackCommand->SetAbortFlag(1);
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
  vtkSeedRepresentation* rep =
    reinterpret_cast<vtkSeedRepresentation*>(this->WidgetRep);
  int activeHandle = rep->GetActiveHandle();
  if (activeHandle < 0 || activeHandle >= rep->GetNumberOfSeeds())
  {
    activeHandle = -1;
  }

  vtkSeedListIterator iter = this->Seeds->begin();
  for (int n = 0; iter != this->Seeds->end(); ++n, ++iter)
  {
    (*iter)->GetHandleRepresentation()->Highlight(
      this->GetEnabled() && n == activeHandle);
  }
  this->InvokeEvent(vtkMaptkSeedWidget::ActiveSeedChangedEvent,
                    &activeHandle);
  this->Render();
}

//----------------------------------------------------------------------
void vtkMaptkSeedWidget::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}
