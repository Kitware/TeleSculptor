/*ckwg +29
 * Copyright 2015-2016 by Kitware, Inc.
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
 *  * Neither name of Kitware, Inc. nor the names of any contributors may be used
 *    to endorse or promote products derived from this software without specific
 *    prior written permission.
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
//#include "am_DepthMapView.h"

#include "ActorColorButton.h"
#include "FeatureOptions.h"
#include "FieldInformation.h"
#include "ImageOptions.h"
#include "vtkMaptkCamera.h"
#include "vtkMaptkFeatureTrackRepresentation.h"

#include <vital/types/landmark_map.h>
#include <vital/types/track.h>

#include <vtkCamera.h>
#include <vtkCellArray.h>
#include <vtkDoubleArray.h>
#include <vtkImageActor.h>
#include <vtkImageData.h>
#include <vtkInteractorStyleRubberBand2D.h>
#include <vtkMatrix4x4.h>
#include <vtkNew.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkUnsignedCharArray.h>
#include <vtkUnsignedIntArray.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkXMLImageDataReader.h>
#include <vtkActor.h>
#include <vtkThreshold.h>
#include <vtkGeometryFilter.h>


#include "DepthMapViewOptions.h"

#include <qtMath.h>
#include <qtUiState.h>

#include <QtGui/QFormLayout>
#include <QtGui/QMenu>
#include <QtGui/QToolButton>
#include <QtGui/QWidgetAction>

QTE_IMPLEMENT_D_FUNC(DepthMapView)

///////////////////////////////////////////////////////////////////////////////

//BEGIN miscelaneous helpers

namespace // anonymous
{

static char const* const TrueColor = "truecolor";
static char const* const Elevation = "elevation";
static char const* const Observations = "observations";

//-----------------------------------------------------------------------------
class ActorColorOption : public QWidget
{
public:
  ActorColorOption(QString const& settingsGroup, QWidget* parent);
  virtual ~ActorColorOption();

  void setDefaultColor(QColor const&);



  ActorColorButton* const button;
  qtUiState uiState;
};

//-----------------------------------------------------------------------------
ActorColorOption::ActorColorOption(
  QString const& settingsGroup, QWidget* parent)
  : QWidget(parent), button(new ActorColorButton(this))
{
  auto const layout = new QFormLayout(this);
  layout->addRow("Color", this->button);

  this->button->persist(this->uiState, settingsGroup + "/Color");
  this->uiState.restore();
}

//-----------------------------------------------------------------------------
ActorColorOption::~ActorColorOption()
{
  this->uiState.save();
}

//-----------------------------------------------------------------------------
void ActorColorOption::setDefaultColor(QColor const& color)
{
  this->button->setColor(color);
  this->uiState.restore();
}

} // namespace <anonymous>

//END miscelaneous helpers

///////////////////////////////////////////////////////////////////////////////

//BEGIN DepthMapViewPrivate definition

//-----------------------------------------------------------------------------
class DepthMapViewPrivate
{
public:

  DepthMapViewPrivate() {}

  Ui::DepthMapView UI;

  vtkNew<vtkRenderer> renderer;
  vtkNew<vtkRenderWindow> renderWindow;

  vtkNew<vtkImageActor> imageActor;
  vtkNew<vtkActor> polyDataActor;
  vtkNew<vtkImageData> imageDM;

  vtkNew<vtkMaptkFeatureTrackRepresentation> featureRep;

  std::map<int, std::string> dMList;

  DepthMapViewOptions* depthMapViewOptions;

  vtkSmartPointer<vtkPolyData> currentDepthmap;

  double imageBounds[6];

  void setPopup(QAction* action, QMenu* menu);
  void setPopup(QAction* action, QWidget* widget);

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

//END DepthMapViewPrivate implementation

///////////////////////////////////////////////////////////////////////////////

//BEGIN DepthMapView

//-----------------------------------------------------------------------------
DepthMapView::DepthMapView(QWidget* parent, Qt::WindowFlags flags)
  : QWidget(parent, flags), d_ptr(new DepthMapViewPrivate)
{
  QTE_D();

  // Set up UI
  d->UI.setupUi(this);

  viewMenu = new QMenu(this);
  viewMenu->addAction(d->UI.actionDisplayMode);

  d->depthMapViewOptions = new DepthMapViewOptions("DepthmapView/DepthmapOptions", this);
  d->setPopup(d->UI.actionDisplayMode, d->depthMapViewOptions);

  d->currentDepthmap = vtkSmartPointer<vtkPolyData>::New();

  // Connect actions
  connect(d->depthMapViewOptions, SIGNAL(modified()),
               d->UI.renderWidget, SLOT(update()));

  // Set up ortho view
  d->renderer->GetActiveCamera()->ParallelProjectionOn();
  d->renderer->GetActiveCamera()->SetClippingRange(1.0, 3.0);
  d->renderer->GetActiveCamera()->SetPosition(0.0, 0.0, 2.0);

  d->imageActor->SetPosition(0.0, 0.0, -0.5);

  // Set up render pipeline
  d->renderer->SetBackground(0.5, 0.5, 0.5);
  d->renderWindow->AddRenderer(d->renderer.GetPointer());
  d->UI.renderWidget->SetRenderWindow(d->renderWindow.GetPointer());

  // Set interactor
  vtkNew<vtkInteractorStyleRubberBand2D> is;
  d->renderWindow->GetInteractor()->SetInteractorStyle(is.GetPointer());

}

void DepthMapView::updateDepthMapThresholds(double bcMin,double bcMax,double urMin,double urMax)
{

  QTE_D();

  double bestCostValueMin = bcMin;
  double bestCostValueMax = bcMax;
  double uniquenessRatioMin = urMin;
  double uniquenessRatioMax = urMax;

  vtkNew<vtkThreshold> thresholdBestCostValues, thresholdUniquenessRatios;

  thresholdBestCostValues->SetInputData(d->currentDepthmap.Get());

  thresholdBestCostValues->SetInputArrayToProcess(0,0,0,vtkDataObject::FIELD_ASSOCIATION_POINTS,"Best Cost Values");
  thresholdBestCostValues->ThresholdBetween(bestCostValueMin,bestCostValueMax);

  thresholdUniquenessRatios->SetInputConnection(thresholdBestCostValues->GetOutputPort());
  thresholdUniquenessRatios->SetInputArrayToProcess(0,0,0,vtkDataObject::FIELD_ASSOCIATION_POINTS,"Uniqueness Ratios");
  thresholdUniquenessRatios->ThresholdBetween(uniquenessRatioMin,uniquenessRatioMax);

  vtkNew<vtkGeometryFilter> geometryFilter;
  geometryFilter->SetInputConnection(thresholdUniquenessRatios->GetOutputPort());
  d->polyDataActor->GetMapper()->SetInputConnection(geometryFilter->GetOutputPort());
  d->polyDataActor->GetMapper()->Update();
}

//-----------------------------------------------------------------------------
DepthMapView::~DepthMapView()
{
}

void DepthMapView::setDepthMap(QString imagePath)
{
  QTE_D();

  if(!imagePath.isEmpty())
  {
    d->depthMapViewOptions->cleanModes();

    vtkNew<vtkXMLImageDataReader> readerIm;

    readerIm->SetFileName(imagePath.toStdString().c_str());
    readerIm->Update();

    vtkNew<vtkGeometryFilter> geometryFilterIm;
    geometryFilterIm->SetInputData(readerIm->GetOutput());
    geometryFilterIm->Update();

    d->UI.toolBar->update();

    d->currentDepthmap = geometryFilterIm->GetOutput();

    vtkNew<vtkPolyDataMapper> mapperP;
    mapperP->SetInputData(geometryFilterIm->GetOutput());
    d->polyDataActor->SetMapper(mapperP.Get());

    d->depthMapViewOptions->addPolyData(d->polyDataActor.Get());

    d->renderer->AddViewProp(d->polyDataActor.GetPointer());

    d->renderer->ResetCamera(readerIm->GetOutput()->GetBounds());

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




//END DepthMapView
