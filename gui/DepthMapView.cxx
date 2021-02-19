/*ckwg +29
 * Copyright 2016-2018 by Kitware, Inc.
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
#include "vtkMaptkScalarDataFilter.h"

#include <vtkCamera.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkGeometryFilter.h>
#include <vtkImageData.h>
#include <vtkInteractorStyleRubberBand2D.h>
#include <vtkMaptkImageDataGeometryFilter.h>
#include <vtkNew.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkThreshold.h>
#include <vtkXMLImageDataReader.h>

#include <QMenu>
#include <QTimer>
#include <QToolButton>
#include <QWidgetAction>

using namespace DepthMapArrays;

QTE_IMPLEMENT_D_FUNC(DepthMapView)

//-----------------------------------------------------------------------------
class DepthMapViewPrivate
{
public:
  DepthMapViewPrivate() : renderQueued(false) {}
  void setPopup(QAction* action, QMenu* menu);
  void setPopup(QAction* action, QWidget* widget);

  bool viewNeedsReset;
  bool validDepthInput;

  Ui::DepthMapView UI;

  vtkNew<vtkRenderer> renderer;
  vtkSmartPointer<vtkRenderWindow> renderWindow;

  vtkNew<vtkMaptkScalarDataFilter> scalarFilter;
  vtkNew<vtkPolyDataMapper> mapper;
  vtkNew<vtkActor> actor;

  DepthMapViewOptions* depthMapViewOptions;

  vtkSmartPointer<vtkMaptkImageDataGeometryFilter> inputDepthGeometryFilter;

  bool renderQueued;
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

  d->viewNeedsReset = true;
  d->validDepthInput = false;

  // Set up UI
  d->UI.setupUi(this);
  d->renderWindow =
    vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();


  d->depthMapViewOptions = new DepthMapViewOptions("DepthMapView", this);
  d->depthMapViewOptions->setActor(d->actor);
  d->setPopup(d->UI.actionDisplayMode, d->depthMapViewOptions);

  connect(d->depthMapViewOptions, &DepthMapViewOptions::modified,
          this, &DepthMapView::render);

  // Connect actions
  this->addAction(d->UI.actionViewReset);

  connect(d->UI.actionViewReset, &QAction::triggered,
          this, &DepthMapView::resetView);

  // Set up ortho view
  d->renderer->GetActiveCamera()->ParallelProjectionOn();
  d->renderer->GetActiveCamera()->SetClippingRange(1.0, 3.0);
  d->renderer->GetActiveCamera()->SetPosition(0.0, 0.0, 2.0);

  // Set up render pipeline
  d->renderer->SetBackground(0, 0, 0);
  d->renderWindow->AddRenderer(d->renderer);
#if VTK_VERSION_MAJOR < 9
  d->UI.renderWidget->SetRenderWindow(d->renderWindow);
#else
  d->UI.renderWidget->setRenderWindow(d->renderWindow);
#endif

  // Set up depth map actor
  d->scalarFilter->SetScalarArrayName(DepthMapArrays::Depth);
  d->mapper->SetInputConnection(d->scalarFilter->GetOutputPort());
  d->actor->SetMapper(d->mapper);
  d->actor->GetProperty()->SetPointSize(2.0);
  d->actor->VisibilityOff();
  d->renderer->AddViewProp(d->actor);

  // Enable antialiasing by default
  d->renderer->UseFXAAOn();

  // Add keyboard actions for increasing and descreasing depth point size
  QAction* actionIncreasePointSize = new QAction(this);
  actionIncreasePointSize->setShortcut(Qt::Key_Plus);
  actionIncreasePointSize->setShortcutContext(Qt::WidgetWithChildrenShortcut);
  d->UI.renderWidget->addAction(actionIncreasePointSize);
  connect(actionIncreasePointSize, &QAction::triggered,
          this, &DepthMapView::increasePointSize);

  QAction* actionDecreasePointSize = new QAction(this);
  actionDecreasePointSize->setShortcut(Qt::Key_Minus);
  actionDecreasePointSize->setShortcutContext(Qt::WidgetWithChildrenShortcut);
  d->UI.renderWidget->addAction(actionDecreasePointSize);
  connect(actionDecreasePointSize, &QAction::triggered,
          this, &DepthMapView::decreasePointSize);

  // Set interactor
  vtkNew<vtkInteractorStyleRubberBand2D> is;
#if VTK_VERSION_MAJOR < 9
  d->UI.renderWidget->GetInteractor()->SetInteractorStyle(is);
#else
  d->UI.renderWidget->interactor()->SetInteractorStyle(is);
#endif
}

//-----------------------------------------------------------------------------
void DepthMapView::updateThresholds()
{
  if (this->isVisible())
  {
    this->render();
  }
}

//-----------------------------------------------------------------------------
DepthMapView::~DepthMapView()
{
}

//-----------------------------------------------------------------------------
void DepthMapView::setValidDepthInput(bool state)
{
  QTE_D();

  d->validDepthInput = state;
  if (!state)
  {
    d->actor->VisibilityOff();
  }
}

//-----------------------------------------------------------------------------
void DepthMapView::updateView(bool processUpdate)
{
  QTE_D();

  if (processUpdate && d->validDepthInput && this->isVisible())
  {
    // Make sure the actor is visible
    d->actor->VisibilityOn();

    // Reset the depth view if the bounds from the geometry filter prior to
    // update are invalid
    bool resetView = false;
    double* bounds = d->inputDepthGeometryFilter->GetOutput()->GetBounds();
    if (bounds[0] > bounds[1])
    {
      resetView = true;
    }

    d->inputDepthGeometryFilter->Update();

    d->depthMapViewOptions->updateRanges(
      d->inputDepthGeometryFilter->GetOutput()->GetPointData());
    d->depthMapViewOptions->updateActor();

    this->render();

    if (resetView || d->viewNeedsReset)
    {
      d->viewNeedsReset = false;
      this->resetView();
    }
  }
}

//-----------------------------------------------------------------------------
void DepthMapView::setBackgroundColor(QColor const& color)
{
  QTE_D();

  d->renderer->SetBackground(color.redF(), color.greenF(), color.blueF());
  this->render();
}

//-----------------------------------------------------------------------------
void DepthMapView::setDepthGeometryFilter(
  vtkMaptkImageDataGeometryFilter* geometryFilter)
{
  QTE_D();

  if (d->inputDepthGeometryFilter != geometryFilter)
  {
    d->inputDepthGeometryFilter = geometryFilter;
    d->scalarFilter->SetInputConnection(d->inputDepthGeometryFilter ?
      d->inputDepthGeometryFilter->GetOutputPort() : 0);
  }
}

//-----------------------------------------------------------------------------
void DepthMapView::resetView()
{
  QTE_D();

  double renderAspect[2];
  d->renderer->GetAspect(renderAspect);

  double bounds[6];
  d->inputDepthGeometryFilter->GetOutput()->GetBounds(bounds);

  auto const w = bounds[1] - bounds[0];
  auto const h = bounds[3] - bounds[4];
  auto const a = w / h;

  auto const s = 0.5 * h * qMax(a / renderAspect[0], 1.0);

  d->renderer->ResetCamera(bounds);
  d->renderer->GetActiveCamera()->SetParallelScale(s);

  this->render();
}

//-----------------------------------------------------------------------------
void DepthMapView::increasePointSize()
{
  QTE_D();

  float pointSize = d->actor->GetProperty()->GetPointSize();
  d->actor->GetProperty()->SetPointSize(pointSize + 0.5);

  this->render();
}

//-----------------------------------------------------------------------------
void DepthMapView::decreasePointSize()
{
  QTE_D();

  float pointSize = d->actor->GetProperty()->GetPointSize() - 0.5;
  d->actor->GetProperty()->SetPointSize(pointSize < 1 ? 1 : pointSize);

  this->render();
}

//-----------------------------------------------------------------------------
void DepthMapView::render()
{
  QTE_D();

  if (!d->renderQueued)
  {
    QTimer::singleShot(0, this, [d]() {
      d->renderWindow->Render();
      d->renderQueued = false;
    });
  }
}

//-----------------------------------------------------------------------------
void DepthMapView::enableAntiAliasing(bool enable)
{
  QTE_D();

  d->renderer->SetUseFXAA(enable);
  this->render();
}
