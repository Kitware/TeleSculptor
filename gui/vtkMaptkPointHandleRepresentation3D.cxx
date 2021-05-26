// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

// TeleSculptor includes
#include "vtkMaptkPointHandleRepresentation3D.h"

// VTK includes
#include <vtkAssembly.h>
#include <vtkCamera.h>
#include <vtkCellPicker.h>
#include <vtkFocalPlanePointPlacer.h>
#include <vtkInteractorObserver.h>
#include <vtkObjectFactory.h>
#include <vtkPlane.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
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

//----------------------------------------------------------------------
// Based on the displacement vector (computed in display coordinates) and
// the cursor state (which corresponds to which part of the widget has been
// selected), the widget points are modified.
// First construct a local coordinate system based on the display coordinates
// of the widget.
void vtkMaptkPointHandleRepresentation3D::WidgetInteraction(double eventPos[2])
{
  // Do different things depending on state
  // Calculations everybody does
  double focalPoint[4], pickPoint[4], prevPickPoint[4], startPickPoint[4], z;

  // Compute the two points defining the motion vector
  vtkInteractorObserver::ComputeWorldToDisplay(this->Renderer,
                                               this->LastPickPosition[0],
                                               this->LastPickPosition[1],
                                               this->LastPickPosition[2],
                                               focalPoint);
  z = focalPoint[2];
  vtkInteractorObserver::ComputeDisplayToWorld(this->Renderer,
                                               this->LastEventPosition[0],
                                               this->LastEventPosition[1],
                                               z,
                                               prevPickPoint);
  vtkInteractorObserver::ComputeDisplayToWorld(
    this->Renderer, eventPos[0], eventPos[1], z, pickPoint);

  // Process the motion
  if (this->InteractionState == vtkHandleRepresentation::Selecting ||
      this->InteractionState == vtkHandleRepresentation::Translating)
  {
    this->WaitCount++;

    if (this->WaitCount > 3 || !this->Constrained)
    {
      vtkInteractorObserver::ComputeDisplayToWorld(this->Renderer,
                                                   this->StartEventPosition[0],
                                                   this->StartEventPosition[1],
                                                   z,
                                                   startPickPoint);

      this->ConstraintAxis = this->DetermineConstraintAxis(
        this->ConstraintAxis, pickPoint, startPickPoint);

      if (this->InteractionState == vtkHandleRepresentation::Selecting &&
          !this->TranslationMode)
      {
        vtkDebugMacro(<< "Processing widget interaction for Select mode");

        // If we are doing axis constrained motion, igonore the placer.
        // Can't have both the placer and an axis constraint dictating
        // handle placement.
        if (this->ConstraintAxis >= 0 || this->Constrained ||
            !this->PointPlacer)
        {
          if (this->CustomConstraint)
          {
            this->TranslateConstrained(prevPickPoint, pickPoint, eventPos);
          }
          else
          {
            this->MoveFocus(prevPickPoint, pickPoint);
          }
        }
        else
        {
          double newCenterPointRequested[3]; // displayPosition
          double newCenterPoint[3], worldOrient[9];

          // Make a request for the new position.
          this->MoveFocusRequest(
            prevPickPoint, pickPoint, eventPos, newCenterPointRequested);

          vtkFocalPlanePointPlacer* fPlacer =
            vtkFocalPlanePointPlacer::SafeDownCast(this->PointPlacer);
          if (fPlacer)
          {
            // Offset the placer plane to one that passes through the current
            // world position and is parallel to the focal plane. Offset =
            // the distance currentWorldPos is from the focal plane
            //
            double currentWorldPos[3], projDir[3], fp[3];
            this->GetWorldPosition(currentWorldPos);
            this->Renderer->GetActiveCamera()->GetFocalPoint(fp);
            double vec[3] = { currentWorldPos[0] - fp[0],
                              currentWorldPos[1] - fp[1],
                              currentWorldPos[2] - fp[2] };
            this->Renderer->GetActiveCamera()->GetDirectionOfProjection(
              projDir);
            fPlacer->SetOffset(vtkMath::Dot(vec, projDir));
          }

          vtkDebugMacro(<< "Request for computing world position at "
                        << "display position of " << newCenterPointRequested[0]
                        << "," << newCenterPointRequested[1]);

          // See what the placer says.
          if (this->PointPlacer->ComputeWorldPosition(this->Renderer,
                                                      newCenterPointRequested,
                                                      newCenterPoint,
                                                      worldOrient))
          {
            // Once the placer has validated us, update the handle position
            this->SetWorldPosition(newCenterPoint);
          }
        }
      }
      else
      {
        vtkDebugMacro(<< "Processing widget interaction for translate");

        // If we are doing axis constrained motion, igonore the placer.
        // Can't have both the placer and the axis constraint dictating
        // handle placement.
        if (this->ConstraintAxis >= 0 || this->Constrained ||
            !this->PointPlacer)
        {
          if (this->CustomConstraint)
          {
            this->TranslateConstrained(prevPickPoint, pickPoint, eventPos);
          }
          else
          {
            this->Translate(prevPickPoint, pickPoint);
          }
        }
        else
        {
          double newCenterPointRequested[3]; // displayPosition
          double newCenterPoint[3], worldOrient[9];

          // Make a request for the new position.
          this->MoveFocusRequest(
            prevPickPoint, pickPoint, eventPos, newCenterPointRequested);

          vtkFocalPlanePointPlacer* fPlacer =
            vtkFocalPlanePointPlacer::SafeDownCast(this->PointPlacer);
          if (fPlacer)
          {
            // Offset the placer plane to one that passes through the current
            // world position and is parallel to the focal plane. Offset =
            // the distance currentWorldPos is from the focal plane
            //
            double currentWorldPos[3], projDir[3], fp[3];
            this->GetWorldPosition(currentWorldPos);
            this->Renderer->GetActiveCamera()->GetFocalPoint(fp);
            double vec[3] = { currentWorldPos[0] - fp[0],
                              currentWorldPos[1] - fp[1],
                              currentWorldPos[2] - fp[2] };
            this->Renderer->GetActiveCamera()->GetDirectionOfProjection(
              projDir);
            fPlacer->SetOffset(vtkMath::Dot(vec, projDir));
          }
          vtkDebugMacro(<< "Request for computing world position at "
                        << "display position of " << newCenterPointRequested[0]
                        << "," << newCenterPointRequested[1]);

          // See what the placer says.
          if (this->PointPlacer->ComputeWorldPosition(this->Renderer,
                                                      newCenterPointRequested,
                                                      newCenterPoint,
                                                      worldOrient))
          {

            // Once the placer has validated us, update the handle
            // position and its bounds.
            double* p = this->GetWorldPosition();

            // Get the motion vector
            double v[3] = { newCenterPoint[0] - p[0],
                            newCenterPoint[1] - p[1],
                            newCenterPoint[2] - p[2] };
            double *bounds = this->Cursor3D->GetModelBounds(), newBounds[6];
            for (int i = 0; i < 3; i++)
            {
              newBounds[2 * i] = bounds[2 * i] + v[i];
              newBounds[2 * i + 1] = bounds[2 * i + 1] + v[i];
            }

            this->Cursor3D->SetModelBounds(newBounds);
            this->SetWorldPosition(newCenterPoint);
          }
        }
      }
    }
  }

  else if (this->InteractionState == vtkHandleRepresentation::Scaling)
  {
    // Scaling does not change the position of the handle, we needn't
    // ask the placer..
    this->Scale(prevPickPoint, pickPoint, eventPos);
  }

  // Book keeping
  this->LastEventPosition[0] = eventPos[0];
  this->LastEventPosition[1] = eventPos[1];

  this->Modified();
}

//----------------------------------------------------------------------------
void vtkMaptkPointHandleRepresentation3D::MoveFocusConstrained(double* p1,
                                                               double* p2)
{
  // Get the motion vector
  double v[3];
  v[0] = p2[0] - p1[0];
  v[1] = p2[1] - p1[1];
  v[2] = p2[2] - p1[2];

  double focus[3];
  this->Cursor3D->GetFocalPoint(focus);
  if (this->ConstraintAxis >= 0)
  {
    focus[this->ConstraintAxis] += v[this->ConstraintAxis];
    if (this->ConstraintAxis < 2)
    {
      focus[1 - this->ConstraintAxis] += v[1 - this->ConstraintAxis];
    }
  }
  else
  {
    focus[0] += v[0];
    focus[1] += v[1];
    focus[2] += v[2];
  }

  this->SetWorldPosition(focus);
}

//----------------------------------------------------------------------------
void vtkMaptkPointHandleRepresentation3D::TranslateConstrained(double* p1,
                                                               double* p2,
                                                               double* ePos)
{

  double normal[3] = { 0, 0, 1 };
  double currentWorldPos[3];
  this->GetWorldPosition(currentWorldPos);

  if (this->ConstraintAxis >= 0)
  {
    if (this->ConstraintAxis == 2)
    {
      // move along the z axis
      double vec[3] = { currentWorldPos[0], currentWorldPos[1], 0.0 };
      double zvec[3] = { 0, 0, 1 };
      vtkMath::Cross(zvec, vec, normal);
    }
  }
  else
  {
    return;
  }
  double nearWorldPoint[4];
  double farWorldPoint[4];
  double tmp[3];

  tmp[0] = ePos[0];
  tmp[1] = ePos[1];
  tmp[2] = 0.0; // near plane

  this->Renderer->SetDisplayPoint(tmp);
  this->Renderer->DisplayToWorld();
  this->Renderer->GetWorldPoint(nearWorldPoint);

  tmp[2] = 1.0; // far plane
  this->Renderer->SetDisplayPoint(tmp);
  this->Renderer->DisplayToWorld();
  this->Renderer->GetWorldPoint(farWorldPoint);

  double position[3];
  double dist;
  if (vtkPlane::IntersectWithLine(
        nearWorldPoint, farWorldPoint, normal, currentWorldPos, dist, position))
  {
    this->SetWorldPosition(position);
  }
  this->LastPickPosition[0] = position[0];
  this->LastPickPosition[1] = position[1];
  this->LastPickPosition[2] = position[2];
  this->Cursor3D->SetFocalPoint(position);
  this->SizeBounds();
}

//----------------------------------------------------------------------
void vtkMaptkPointHandleRepresentation3D::BuildRepresentation()
{
  // The net effect is to resize the handle
  if (this->GetMTime() > this->BuildTime ||
      (this->Renderer && this->Renderer->GetVTKWindow() &&
       this->Renderer->GetVTKWindow()->GetMTime() > this->BuildTime))
  {
    if (!this->Placed)
    {
      this->ValidPick = 1;
      this->Placed = 1;
    }

    this->SizeBounds();
    this->Cursor3D->Update();
    this->UpdateAxes();
    this->BuildTime.Modified();
  }
}

//----------------------------------------------------------------------
void vtkMaptkPointHandleRepresentation3D::UpdateAxes()
{
  if (!this->Renderer)
  {
    return;
  }

  if (!this->Renderer->HasViewProp(this->AxesActor))
  {
    this->Renderer->AddActor(this->AxesActor);
  }
  if (!this->CustomConstraint || !this->Constrained ||
      this->ConstraintAxis < 0 || this->ConstraintAxis > 2)
  {
    this->Renderer->RemoveViewProp(this->AxesActor);
    return;
  }

  double bounds[6], center[3];
  this->Cursor3D->GetFocalPoint(center);
  this->Cursor3D->GetModelBounds(bounds);

  this->AxesActor->SetPosition(center);
  if (this->ConstraintAxis == 0 || this->ConstraintAxis == 1)
  {
    this->AxesActor->SetZAxisVisibility(0);
    this->AxesActor->SetXYPlaneVisibility(1);
  }
  else
  {
    this->AxesActor->SetZAxisVisibility(1);
    this->AxesActor->SetXYPlaneVisibility(0);
  }
  this->AxesActor->SetAxesLength(3 * (bounds[1] - bounds[0]));
}

//----------------------------------------------------------------------
void vtkMaptkPointHandleRepresentation3D::ShallowCopy(vtkProp* p)
{
  vtkMaptkPointHandleRepresentation3D* rep =
    vtkMaptkPointHandleRepresentation3D::SafeDownCast(p);
  if (p)
  {
    this->SetCustomConstraint(rep->GetCustomConstraint());
    this->SetConstraintAxis(rep->GetConstraintAxis());
  }
  Superclass::ShallowCopy(p);
}

//----------------------------------------------------------------------
void vtkMaptkPointHandleRepresentation3D::DeepCopy(vtkProp* p)
{
  vtkMaptkPointHandleRepresentation3D* rep =
    vtkMaptkPointHandleRepresentation3D::SafeDownCast(p);
  if (p)
  {
    this->SetCustomConstraint(rep->GetCustomConstraint());
    this->SetConstraintAxis(rep->GetConstraintAxis());
  }
  Superclass::DeepCopy(p);
}
