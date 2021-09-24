// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

// TeleSculptor includes
#include "GroundControlPointsWidget.h"
#include "vtkMaptkPointHandleRepresentation3D.h"
#include "vtkMaptkSeedWidget.h"

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
#include <QSignalBlocker>

QTE_IMPLEMENT_D_FUNC(GroundControlPointsWidget)

//-----------------------------------------------------------------------------
class GroundControlPointsWidgetPrivate
{
public:
  GroundControlPointsWidgetPrivate();

  // Methods
  void updateTransformMatrixInverse();
  void deletePoint(int handleId);

  // Seed widget
  vtkNew<vtkMaptkSeedWidget> widget;
  vtkNew<vtkSeedRepresentation> repr;
  vtkNew<vtkEventQtSlotConnect> connections;
  vtkNew<vtkMaptkPointHandleRepresentation3D> pointRepr;

  vtkRenderer* renderer = nullptr;

  vtkMatrix4x4* transformMatrix = nullptr;
  vtkNew<vtkMatrix4x4> transformMatrixInverse;
  vtkMTimeType transformMatrixMTime = 0;
};

//-----------------------------------------------------------------------------
GroundControlPointsWidgetPrivate::GroundControlPointsWidgetPrivate()
{
  this->widget->SetRepresentation(this->repr.GetPointer());
  this->widget->SetEnabled(0);

  // Set up the representation
  this->widget->ManagesCursorOn();
  this->repr->SetHandleRepresentation(this->pointRepr.GetPointer());
  vtkNew<vtkProperty> property;
  property->SetLineWidth(1.0);
  property->SetRenderLinesAsTubes(false);
  this->pointRepr->SetProperty(property.GetPointer());
  vtkNew<vtkProperty> selectedProperty;
  selectedProperty->SetLineWidth(3.0);
  selectedProperty->SetRenderLinesAsTubes(true);
  this->pointRepr->SetSelectedProperty(selectedProperty.GetPointer());

  this->widget->GetEventTranslator()->RemoveTranslation(
    vtkCommand::RightButtonPressEvent);
}

//-----------------------------------------------------------------------------
void GroundControlPointsWidgetPrivate::updateTransformMatrixInverse()
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
void GroundControlPointsWidgetPrivate::deletePoint(int handleId)
{
  if (this->renderer)
  {
    this->widget->DeleteSeed(handleId);
    this->repr->SetActiveHandle(handleId - 1);
  }
}

//-----------------------------------------------------------------------------
GroundControlPointsWidget::GroundControlPointsWidget(QObject* parent)
  : QObject(parent)
  , d_ptr(new GroundControlPointsWidgetPrivate)
{
  QTE_D();

  this->setColor(Qt::white);
  this->setSelectedColor(Qt::green);

  d->connections->Connect(d->widget.GetPointer(),
                          vtkCommand::PlacePointEvent,
                          this,
                          SIGNAL(pointPlaced()));
  d->connections->Connect(d->widget.GetPointer(),
                          vtkCommand::InteractionEvent,
                          this,
                          SLOT(movePointEvent()));

  d->connections->Connect(
    d->widget.GetPointer(),
    vtkCommand::DeletePointEvent,
    this,
    SLOT(pointDeletedCallback(vtkObject*, unsigned long, void*, void*)));
  d->connections->Connect(
    d->widget.GetPointer(),
    vtkMaptkSeedWidget::ActiveSeedChangedEvent,
    this,
    SLOT(activeHandleChangedCallback(vtkObject*, unsigned long, void*, void*)));
}

//-----------------------------------------------------------------------------
GroundControlPointsWidget::~GroundControlPointsWidget()
{
}

//-----------------------------------------------------------------------------
void GroundControlPointsWidget::enableWidget(bool enable)
{
  QTE_D();

  with_expr (QSignalBlocker{this})
  {
    d->widget->SetEnabled(enable);
    d->widget->GetInteractor()->Render();
  }
}

//-----------------------------------------------------------------------------
void GroundControlPointsWidget::setInteractor(vtkRenderWindowInteractor* iren)
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
void GroundControlPointsWidget::setPointPlacer(vtkPointPlacer* placer)
{
  QTE_D();
  d->pointRepr->SetPointPlacer(placer);
}

//-----------------------------------------------------------------------------
void GroundControlPointsWidget::setColor(QColor color)
{
  QTE_D();

  auto* const property = d->pointRepr->GetProperty();
  property->SetColor(color.redF(), color.greenF(), color.blueF());
}

//-----------------------------------------------------------------------------
void GroundControlPointsWidget::setSelectedColor(QColor color)
{
  QTE_D();

  auto* const property = d->pointRepr->GetSelectedProperty();
  property->SetColor(color.redF(), color.greenF(), color.blueF());
}

//-----------------------------------------------------------------------------
vtkRenderer* GroundControlPointsWidget::renderer()
{
  QTE_D();
  return d->renderer;
}

//-----------------------------------------------------------------------------
int GroundControlPointsWidget::activeHandle() const
{
  QTE_D();

  return d->repr->GetActiveHandle();
}

//-----------------------------------------------------------------------------
kwiver::vital::vector_3d GroundControlPointsWidget::activePoint()
{
  QTE_D();

  double pt[4] = { 0, 0, 0, 1 };
  d->repr->GetSeedWorldPosition(this->activeHandle(), pt);
  if (d->transformMatrix)
  {
    d->updateTransformMatrixInverse();
    d->transformMatrixInverse->MultiplyPoint(pt, pt);
  }

  return kwiver::vital::vector_3d(pt[0] / pt[3], pt[1] / pt[3], pt[2] / pt[3]);
}

//-----------------------------------------------------------------------------
kwiver::vital::vector_3d GroundControlPointsWidget::point(int handle) const
{
  QTE_D();
  kwiver::vital::vector_3d out;
  d->repr->GetSeedWorldPosition(handle, out.data());
  return out;
}

//-----------------------------------------------------------------------------
vtkHandleWidget* GroundControlPointsWidget::handleWidget(int handleId) const
{
  QTE_D();

  return d->widget->GetSeed(handleId);
}

//-----------------------------------------------------------------------------
int GroundControlPointsWidget::findHandleWidget(vtkHandleWidget* handle) const
{
  QTE_D();
  return d->widget->FindSeed(handle);
}

//-----------------------------------------------------------------------------
int GroundControlPointsWidget::addDisplayPoint(double pt[3])
{
  QTE_D();

  if (!d->renderer)
  {
    return -1;
  }

  auto const handleId = static_cast<unsigned>(d->repr->CreateHandle(pt));
  d->repr->SetSeedWorldPosition(handleId, pt);
  // Now that the seed is placed, reset the point placer to ensure free
  // motion of the handle
  d->repr->GetHandleRepresentation(handleId)->SetPointPlacer(nullptr);
  vtkHandleWidget* currentHandle = d->widget->CreateNewHandle();
  currentHandle->SetEnabled(1);

  return static_cast<int>(handleId);
}

//-----------------------------------------------------------------------------
int GroundControlPointsWidget::addDisplayPoint(double x, double y, double z)
{
  double pt[3] = { x, y, z };
  return this->addDisplayPoint(pt);
}

//-----------------------------------------------------------------------------
int GroundControlPointsWidget::addDisplayPoint(kwiver::vital::vector_3d pt)
{
  return this->addDisplayPoint(pt.data());
}

//-----------------------------------------------------------------------------
int GroundControlPointsWidget::addPoint(double x, double y, double z)
{
  QTE_D();

  if (!d->renderer)
  {
    return -1;
  }

  double p[4] = { x, y, z, 1.0 };
  if (d->transformMatrix)
  {
    d->transformMatrix->MultiplyPoint(p, p);
  }

  auto const handleId = this->addDisplayPoint(p);

  d->widget->HighlightActiveSeed();

  return handleId;
}

//-----------------------------------------------------------------------------
int GroundControlPointsWidget::addPoint(double const p[3])
{
  return this->addPoint(p[0], p[1], p[2]);
}

//-----------------------------------------------------------------------------
int GroundControlPointsWidget::addPoint(kwiver::vital::vector_3d p)
{
  return this->addPoint(p.data());
}

//-----------------------------------------------------------------------------
void GroundControlPointsWidget::movePoint(int handleId,
                                          double x,
                                          double y,
                                          double z)
{
  QTE_D();

  if (this->numberOfPoints() <= handleId)
  {
    return;
  }

  double p[4] = { x, y, z, 1.0 };
  if (d->transformMatrix)
  {
    d->transformMatrix->MultiplyPoint(p, p);
  }

  if (d->renderer)
  {
    d->repr->SetSeedWorldPosition(handleId, p);
  }
}

//-----------------------------------------------------------------------------
void GroundControlPointsWidget::movePointEvent()
{
  QTE_D();

  if (d->widget->GetWidgetState() == vtkMaptkSeedWidget::MovingSeed)
  {
    emit pointMoved();
  }
}

//-----------------------------------------------------------------------------
void GroundControlPointsWidget::deletePoint(int handleId)
{
  QTE_D();
  d->deletePoint(handleId);
  d->widget->GetInteractor()->Render();
}

//-----------------------------------------------------------------------------
void GroundControlPointsWidget::pointDeletedCallback(
  vtkObject* caller,
  unsigned long vtkNotUsed(event),
  void* vtkNotUsed(clientData),
  void* callData)
{
  int* handleId = reinterpret_cast<int*>(callData);
  emit pointDeleted(*handleId);
}

//-----------------------------------------------------------------------------
void GroundControlPointsWidget::setTransformMatrix(vtkMatrix4x4* m)
{
  QTE_D();
  d->transformMatrix = m;
  d->updateTransformMatrixInverse();
}

//-----------------------------------------------------------------------------
vtkMatrix4x4* GroundControlPointsWidget::transformMatrix() const
{
  QTE_D();
  return d->transformMatrix;
}

//-----------------------------------------------------------------------------
int GroundControlPointsWidget::numberOfPoints() const
{
  QTE_D();

  return d->repr->GetNumberOfSeeds();
}

//-----------------------------------------------------------------------------
void GroundControlPointsWidget::render()
{
  QTE_D();
  d->widget->GetInteractor()->Render();
}

//-----------------------------------------------------------------------------
void GroundControlPointsWidget::clearPoints()
{
  QTE_D();

  if (d->renderer)
  {
    int n = this->numberOfPoints();
    while (n)
    {
      d->deletePoint(--n);
    }
    d->widget->GetInteractor()->Render();
  }
}

//-----------------------------------------------------------------------------
void GroundControlPointsWidget::activeHandleChangedCallback(
  vtkObject* vtkNotUsed(ob),
  unsigned long vtkNotUsed(e),
  void* vtkNotUsed(cD),
  void* callData)
{
  int* handleId = reinterpret_cast<int*>(callData);
  if (handleId)
  {
    emit this->activePointChanged(*handleId);
  }
}

//-----------------------------------------------------------------------------
void GroundControlPointsWidget::setActivePoint(int id)
{
  QTE_D();

  if (!d->repr->GetNumberOfSeeds())
  {
    return;
  }

  if (d->repr->GetActiveHandle() == id)
  {
    return;
  }
  d->repr->SetActiveHandle(id);
  d->widget->HighlightActiveSeed();
}
