/*ckwg +29
 * Copyright 2018 by Kitware, Inc.
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
#include "RulerWidget.h"
#include "vtkMaptkDistanceRepresentation2D.h"
#include "vtkMaptkDistanceWidget.h"
#include "vtkMaptkPointHandleRepresentation3D.h"
#include "vtkMaptkPointPlacer.h"

// VTK includes
#include <vtkActor.h>
#include <vtkCommand.h>
#include <vtkEvent.h>
#include <vtkEventQtSlotConnect.h>
#include <vtkHandleWidget.h>
#include <vtkMatrix4x4.h>
#include <vtkNew.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSeedRepresentation.h>
#include <vtkWidgetEvent.h>
#include <vtkWidgetEventTranslator.h>

// Qt includes
#include <QApplication>

QTE_IMPLEMENT_D_FUNC(RulerWidget)

//-----------------------------------------------------------------------------
class RulerWidgetPrivate
{
public:
  RulerWidgetPrivate();

  // Methods
  void updateTransformMatrixInverse();

  // Distance widget
  vtkNew<vtkMaptkDistanceWidget> widget;
  vtkNew<vtkMaptkDistanceRepresentation2D> repr;
  vtkNew<vtkEventQtSlotConnect> connections;
  vtkNew<vtkMaptkPointHandleRepresentation3D> pointRepr;

  vtkRenderer* renderer = nullptr;

  vtkMatrix4x4* transformMatrix = nullptr;
  vtkNew<vtkMatrix4x4> transformMatrixInverse;
  vtkMTimeType transformMatrixMTime = 0;

  kwiver::vital::vector_3d invTransformPoint(double* pt);
  void transformPoint(double* pt);
};

//-----------------------------------------------------------------------------
RulerWidgetPrivate::RulerWidgetPrivate()
{
  this->widget->SetRepresentation(this->repr.GetPointer());
  this->widget->SetEnabled(0);

  // Set up the representation
  this->widget->ManagesCursorOn();
  this->repr->SetHandleRepresentation(this->pointRepr.GetPointer());
  this->repr->SetLabelFormat("%-#6.2f");
  vtkNew<vtkProperty> property;
  property->SetColor(1, 1, 1);
  property->SetLineWidth(1.0);
  property->SetRenderLinesAsTubes(false);
  this->pointRepr->SetProperty(property.GetPointer());
  vtkNew<vtkProperty> selectedProperty;
  selectedProperty->SetColor(0, 1, 0);
  selectedProperty->SetLineWidth(3.0);
  selectedProperty->SetRenderLinesAsTubes(true);
  this->pointRepr->SetSelectedProperty(selectedProperty.GetPointer());
}

//-----------------------------------------------------------------------------
void RulerWidgetPrivate::updateTransformMatrixInverse()
{
  if (!this->transformMatrix)
  {
    return;
  }

  if (this->transformMatrix->GetMTime() <= this->transformMatrixMTime)
  {
    return;
  }

  vtkMatrix4x4::Invert(this->transformMatrix,
                       this->transformMatrixInverse.GetPointer());
  this->transformMatrixMTime = this->transformMatrix->GetMTime();
}

//-----------------------------------------------------------------------------
kwiver::vital::vector_3d RulerWidgetPrivate::invTransformPoint(double* pt)
{
  if (this->transformMatrix)
  {
    this->updateTransformMatrixInverse();
    this->transformMatrixInverse->MultiplyPoint(pt, pt);
    pt[0] /= pt[3];
    pt[1] /= pt[3];
    pt[2] /= pt[3];
  }
  return kwiver::vital::vector_3d(pt[0], pt[1], pt[2]);
}

//-----------------------------------------------------------------------------
void RulerWidgetPrivate::transformPoint(double* pt)
{
  if (this->transformMatrix)
  {
    this->transformMatrix->MultiplyPoint(pt, pt);
    pt[0] /= pt[3];
    pt[1] /= pt[3];
    pt[2] /= pt[3];
  }
}

//-----------------------------------------------------------------------------
RulerWidget::RulerWidget(QObject* parent)
  : QObject(parent)
  , d_ptr(new RulerWidgetPrivate)
{
  QTE_D();

  d->connections->Connect(
    d->widget,
    vtkCommand::PlacePointEvent,
    this,
    SLOT(placePoint(vtkObject*, unsigned long, void*, void*)));
  d->connections->Connect(
    d->widget,
    d->widget->DistanceInteractionEvent,
    this,
    SLOT(movePointEvent(vtkObject*, unsigned long, void*, void*)));
  d->connections->Connect(
    d->widget,
    vtkCommand::InteractionEvent,
    this,
    SLOT(movePointEvent(vtkObject*, unsigned long, void*, void*)));
}

//-----------------------------------------------------------------------------
RulerWidget::~RulerWidget()
{
}

//-----------------------------------------------------------------------------
void RulerWidget::enableWidget(bool enable)
{
  QTE_D();
  d->widget->SetEnabled(enable);
  d->widget->GetInteractor()->Render();
}

//-----------------------------------------------------------------------------
void RulerWidget::setInteractor(vtkRenderWindowInteractor* iren)
{
  QTE_D();
  d->widget->SetInteractor(iren);

  // Compute an appropriate scale factor for the glyphs
  vtkRenderer* ren = iren->FindPokedRenderer(0, 0);
  if (ren)
  {
    d->renderer = ren;
  }
}

//-----------------------------------------------------------------------------
void RulerWidget::setPointPlacer(vtkPointPlacer* placer)
{
  QTE_D();
  d->pointRepr->SetPointPlacer(placer);
}

//-----------------------------------------------------------------------------
void RulerWidget::placePoint(vtkObject* vtkNotUsed(ob),
                             unsigned long vtkNotUsed(evId),
                             void* vtkNotUsed(clientData),
                             void* callData)
{
  QTE_D();
  int* handleId = reinterpret_cast<int*>(callData);
  if (handleId)
  {
    // adjust point based on point placer
    if (vtkMaptkPointPlacer::SafeDownCast(d->pointRepr->GetPointPlacer()))
    {
      switch (*handleId)
      {
        case 0:
        {
          double pos[3], newPos[3];
          d->repr->GetPoint1DisplayPosition(pos);
          d->pointRepr->GetPointPlacer()->ComputeWorldPosition(
            this->renderer(), pos, newPos, nullptr);
          d->repr->SetPoint1WorldPosition(newPos);
          break;
        }
        case 1:
        {
          double pos[3], newPos[3];
          d->repr->GetPoint2DisplayPosition(pos);
          d->pointRepr->GetPointPlacer()->ComputeWorldPosition(
            this->renderer(), pos, newPos, nullptr);
          d->repr->SetPoint2WorldPosition(newPos);
          break;
        }
        default:
        {
          break;
        }
      }
    }
    emit this->pointPlaced(*handleId);
  }
}

//-----------------------------------------------------------------------------
vtkRenderer* RulerWidget::renderer()
{
  QTE_D();
  return d->renderer;
}

//-----------------------------------------------------------------------------
void RulerWidget::movePointEvent(vtkObject* vtkNotUsed(da),
                                 unsigned long vtkNotUsed(evId),
                                 void* vtkNotUsed(clientData),
                                 void* callData)
{
  QTE_D();

  if (d->widget->GetWidgetState() != vtkMaptkDistanceWidget::Define)
  {
    // Got here via DistanceInteractionEvent or InteractionEvent
    int* handleId = reinterpret_cast<int*>(callData);
    if (handleId)
    {
      // Do not fire signal if the handleId is not know i.e. if we got here from
      // the superclass' interaction event.
      emit pointMoved(*handleId);
    }
  }
  else
  {
    // Got here via InteractionEvent while defining the second point
    emit pointPlaced(1);
  }
}

//-----------------------------------------------------------------------------
void RulerWidget::setTransformMatrix(vtkMatrix4x4* m)
{
  QTE_D();
  d->transformMatrix = m;
  d->updateTransformMatrixInverse();
}

//-----------------------------------------------------------------------------
vtkMatrix4x4* RulerWidget::transformMatrix() const
{
  QTE_D();
  return d->transformMatrix;
}

//-----------------------------------------------------------------------------
void RulerWidget::render()
{
  QTE_D();
  d->widget->GetInteractor()->Render();
}

//-----------------------------------------------------------------------------
void RulerWidget::setPoint1WorldPosition(double* pos)
{
  this->setPoint1WorldPosition(pos[0], pos[1], pos[2]);
}

//-----------------------------------------------------------------------------
void RulerWidget::setPoint1WorldPosition(double x, double y, double z)
{
  QTE_D();
  double pos[4] = { x, y, z, 1.0 };
  d->transformPoint(pos);
  d->repr->SetPoint1WorldPosition(pos);
}

//-----------------------------------------------------------------------------
kwiver::vital::vector_3d RulerWidget::point1WorldPosition()
{
  QTE_D();
  double pt[4] = { 0, 0, 0, 1 };
  if (d->repr->GetPoint1Representation())
  {
    d->repr->GetPoint1WorldPosition(pt);
  }
  return d->invTransformPoint(pt);
}

//-----------------------------------------------------------------------------
void RulerWidget::setPoint2WorldPosition(double* pos)
{
  this->setPoint2WorldPosition(pos[0], pos[1], pos[2]);
}

//-----------------------------------------------------------------------------
void RulerWidget::setPoint2WorldPosition(double x, double y, double z)
{
  QTE_D();
  double pos[4] = { x, y, z, 1.0 };
  d->transformPoint(pos);
  d->repr->SetPoint2WorldPosition(pos);
  d->repr->VisibilityOn();
  d->widget->SetWidgetStateToManipulate();
  d->widget->SetEnabled(1);
}

//-----------------------------------------------------------------------------
kwiver::vital::vector_3d RulerWidget::point2WorldPosition()
{
  QTE_D();
  double pt[4] = { 0, 0, 0, 1 };
  if (d->repr->GetPoint2Representation())
  {
    d->repr->GetPoint2WorldPosition(pt);
  }
  return d->invTransformPoint(pt);
}

//-----------------------------------------------------------------------------
void RulerWidget::setComputeDistance(bool compute)
{
  QTE_D();
  d->repr->SetComputeDistance(compute);
}

//-----------------------------------------------------------------------------
void RulerWidget::setDistance(double distance)
{
  QTE_D();
  d->repr->SetDistance(distance);
}

//-----------------------------------------------------------------------------
double RulerWidget::distance() const
{
  QTE_D();
  return d->repr->GetDistance();
}
