/*ckwg +29
 * Copyright 2016 by Kitware, Inc.
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

#include "DepthMapView.h"

#include "ui_DepthMapView.h"

#include "DataArrays.h"
#include "DepthMapViewOptions.h"

#include <vtkCamera.h>
#include <vtkGeometryFilter.h>
#include <vtkImageData.h>
#include <vtkInteractorStyleRubberBand2D.h>
#include <vtkNew.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkSmartPointer.h>
#include <vtkThreshold.h>
#include <vtkXMLImageDataReader.h>

#include <QtGui/QMenu>
#include <QtGui/QToolButton>
#include <QtGui/QWidgetAction>

using namespace DepthMapArrays;

QTE_IMPLEMENT_D_FUNC(DepthMapView)

//-----------------------------------------------------------------------------
class DepthMapViewPrivate
{
public:
  void setPopup(QAction* action, QMenu* menu);
  void setPopup(QAction* action, QWidget* widget);

  Ui::DepthMapView UI;

  vtkNew<vtkRenderer> renderer;
  vtkNew<vtkRenderWindow> renderWindow;

  vtkNew<vtkActor> polyDataActor;

  DepthMapViewOptions* depthMapViewOptions;

  vtkSmartPointer<vtkPolyData> currentDepthMap;
};

//-----------------------------------------------------------------------------
void DepthMapViewPrivate::setPopup(QAction* action, QMenu* menu)
{
  auto const widget = this->UI.toolBar->widgetForAction(action);
  auto const button = qobject_cast<QToolButton*>(widget);

  if (button)
  {
    button->setPopupMode(QToolButton::MenuButtonPopup);
    button->setMenu(menu);
  }
}

//-----------------------------------------------------------------------------
void DepthMapViewPrivate::setPopup(QAction* action, QWidget* widget)
{
  auto const parent = action->parentWidget();

  auto const proxy = new QWidgetAction(parent);
  proxy->setDefaultWidget(widget);

  auto const menu = new QMenu(parent);
  menu->addAction(proxy);

  this->setPopup(action, menu);
}

//-----------------------------------------------------------------------------
DepthMapView::DepthMapView(QWidget* parent, Qt::WindowFlags flags)
  : QWidget(parent, flags), d_ptr(new DepthMapViewPrivate)
{
  QTE_D();

  // Set up UI
  d->UI.setupUi(this);

  d->depthMapViewOptions = new DepthMapViewOptions("DepthMapView", this);
  d->setPopup(d->UI.actionDisplayMode, d->depthMapViewOptions);

  d->currentDepthMap = vtkSmartPointer<vtkPolyData>::New();

  // Connect actions
  this->addAction(d->UI.actionViewReset);

  connect(d->UI.actionViewReset, SIGNAL(triggered()),
          this, SLOT(resetView()));
  connect(d->depthMapViewOptions, SIGNAL(modified()),
          d->UI.renderWidget, SLOT(update()));

  // Set up ortho view
  d->renderer->GetActiveCamera()->ParallelProjectionOn();
  d->renderer->GetActiveCamera()->SetClippingRange(1.0, 3.0);
  d->renderer->GetActiveCamera()->SetPosition(0.0, 0.0, 2.0);

  // Set up render pipeline
  d->renderer->SetBackground(0, 0, 0);
  d->renderWindow->AddRenderer(d->renderer.GetPointer());
  d->UI.renderWidget->SetRenderWindow(d->renderWindow.GetPointer());

  // Set interactor
  vtkNew<vtkInteractorStyleRubberBand2D> is;
  d->renderWindow->GetInteractor()->SetInteractorStyle(is.GetPointer());
}

//-----------------------------------------------------------------------------
void DepthMapView::updateThresholds(double bcMin, double bcMax,
                                    double urMin, double urMax)
{
  QTE_D();

  double bestCostValueMin = bcMin;
  double bestCostValueMax = bcMax;
  double uniquenessRatioMin = urMin;
  double uniquenessRatioMax = urMax;

  vtkNew<vtkThreshold> thresholdBestCostValues;
  vtkNew<vtkThreshold> thresholdUniquenessRatios;

  thresholdBestCostValues->SetInputData(d->currentDepthMap.GetPointer());
  thresholdBestCostValues->SetInputArrayToProcess(
    0, 0, 0, vtkDataObject::FIELD_ASSOCIATION_POINTS,
    DepthMapArrays::BestCostValues);
  thresholdBestCostValues->ThresholdBetween(
    bestCostValueMin, bestCostValueMax);

  thresholdUniquenessRatios->SetInputConnection(
    thresholdBestCostValues->GetOutputPort());
  thresholdUniquenessRatios->SetInputArrayToProcess(
    0, 0, 0, vtkDataObject::FIELD_ASSOCIATION_POINTS,
    DepthMapArrays::UniquenessRatios);
  thresholdUniquenessRatios->ThresholdBetween(
    uniquenessRatioMin, uniquenessRatioMax);

  vtkNew<vtkGeometryFilter> geometryFilter;

  geometryFilter->SetInputConnection(
    thresholdUniquenessRatios->GetOutputPort());

  d->polyDataActor->GetMapper()->SetInputConnection(
    geometryFilter->GetOutputPort());
  d->polyDataActor->GetMapper()->Update();

  d->UI.renderWidget->update();
}

//-----------------------------------------------------------------------------
DepthMapView::~DepthMapView()
{
}

//-----------------------------------------------------------------------------
void DepthMapView::setDepthMap(QString const& imagePath)
{
  QTE_D();

  if (!imagePath.isEmpty() && this->isVisible())
  {
    d->depthMapViewOptions->cleanModes();

    vtkNew<vtkXMLImageDataReader> reader;

    reader->SetFileName(qPrintable(imagePath));
    reader->Update();

    vtkNew<vtkGeometryFilter> geometryFilter;
    geometryFilter->SetInputData(reader->GetOutput());
    geometryFilter->Update();

    d->UI.toolBar->update();

    d->currentDepthMap = geometryFilter->GetOutput();

    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputData(geometryFilter->GetOutput());
    d->polyDataActor->SetMapper(mapper.GetPointer());

    d->depthMapViewOptions->addActor(d->polyDataActor.GetPointer());

    d->renderer->AddViewProp(d->polyDataActor.GetPointer());

    d->UI.renderWidget->update();
  }
}

//-----------------------------------------------------------------------------
void DepthMapView::setBackgroundColor(QColor const& color)
{
  QTE_D();

  d->renderer->SetBackground(color.redF(), color.greenF(), color.blueF());
  d->UI.renderWidget->update();
}

//-----------------------------------------------------------------------------
void DepthMapView::resetView()
{
  QTE_D();

  double renderAspect[2];
  d->renderer->GetAspect(renderAspect);

  double bounds[6];
  d->currentDepthMap->GetBounds(bounds);

  auto const w = bounds[1] - bounds[0];
  auto const h = bounds[3] - bounds[4];
  auto const a = w / h;

  auto const s = 0.5 * h * qMax(a / renderAspect[0], 1.0);

  d->renderer->ResetCamera(bounds);
  d->renderer->GetActiveCamera()->SetParallelScale(s);

  d->UI.renderWidget->update();
}
