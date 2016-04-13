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
#include <vtkActor.h>

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

  DepthMapViewPrivate() : featuresDirty(false) {}

  Ui::DepthMapView UI;

  vtkNew<vtkRenderer> renderer;
  vtkNew<vtkRenderWindow> renderWindow;

  vtkNew<vtkImageActor> imageActor;
  vtkNew<vtkActor> polyDataActor;
  vtkNew<vtkImageData> emptyImage;

  vtkNew<vtkMaptkFeatureTrackRepresentation> featureRep;

  double imageBounds[6];

  bool featuresDirty;
};

//END DepthMapViewPrivate definition

///////////////////////////////////////////////////////////////////////////////


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

  this->addAction(d->UI.actionViewReset);
  this->addAction(d->UI.actionViewResetFullExtents);

  auto const viewMenu = new QMenu(this);
  viewMenu->addAction(d->UI.actionViewReset);
  viewMenu->addAction(d->UI.actionViewResetFullExtents);

  // Connect actions
  this->addAction(d->UI.actionViewReset);
  this->addAction(d->UI.actionViewResetFullExtents);

  // Set up ortho view
  d->renderer->GetActiveCamera()->ParallelProjectionOn();
  d->renderer->GetActiveCamera()->SetClippingRange(1.0, 3.0);
  d->renderer->GetActiveCamera()->SetPosition(0.0, 0.0, 2.0);

  //Set up polydata
  vtkNew<vtkXMLPolyDataReader> reader;
  reader->SetFileName("/home/louis/develop/PlaneSweepLib/build/bin/depthMaps/frame_0003_depth_map.0.vtp");
  reader->Update();

  if(reader->GetOutput()) {
    std::cout << "polydata loaded " <<  reader->GetOutput()->GetNumberOfPoints()<< std::endl;
  }

  std::vector<vtkIdType> vertices(reader->GetOutput()->GetNumberOfPoints());
  for (int i = 0; i < reader->GetOutput()->GetNumberOfPoints(); ++i) {
    vertices[i] = i;
  }

  vtkNew<vtkCellArray> cells;

  cells->InsertNextCell(vertices.size(), &vertices[0]);

  reader->GetOutput()->SetVerts(cells.Get());


  reader->GetOutput()->GetPointData()->SetScalars(reader->GetOutput()->GetPointData()->GetArray("Color"));
  vtkNew<vtkPolyDataMapper> mapper;
//  mapper->SetInputConnection(reader->GetOutputPort());
  mapper->SetInputData(reader->GetOutput());
  mapper->SetColorModeToDirectScalars();


  d->polyDataActor->SetMapper(mapper.Get());
  d->polyDataActor->SetVisibility(true);
  d->polyDataActor->GetProperty()->SetPointSize(2);
  d->renderer->AddActor(d->polyDataActor.Get());

  d->renderer->AddViewProp(d->polyDataActor.GetPointer());
  d->imageActor->SetPosition(0.0, 0.0, -0.5);
  // Set up render pipeline
  d->renderer->SetBackground(0.5, 0.5, 0.5);
  d->renderWindow->AddRenderer(d->renderer.GetPointer());
  d->UI.renderWidget->SetRenderWindow(d->renderWindow.GetPointer());
//  d->UI.renderWidget->update();

  // Set interactor
  vtkNew<vtkInteractorStyleRubberBand2D> is;
  d->renderWindow->GetInteractor()->SetInteractorStyle(is.GetPointer());



  // Set up actors
//  d->renderer->AddActor(d->featureRep->GetActivePointsActor());
//  d->renderer->AddActor(d->featureRep->GetTrailsActor());
//  d->renderer->AddActor(d->landmarks.actor.GetPointer());
//  d->renderer->AddActor(d->residuals.actor.GetPointer());

//  d->renderer->AddViewProp(d->imageActor.GetPointer());
//  d->imageActor->SetPosition(0.0, 0.0, -0.5);

  // Create "dummy" image data for use when we have no "real" image
//  d->emptyImage->SetExtent(0, 0, 0, 0, 0, 0);
//  d->emptyImage->AllocateScalars(VTK_UNSIGNED_CHAR, 1);
//  d->emptyImage->SetScalarComponentFromDouble(0, 0, 0, 0, 0.0);

//  this->setImageData(0, QSize(1, 1));
}

//-----------------------------------------------------------------------------
DepthMapView::~DepthMapView()
{
}

//-----------------------------------------------------------------------------
void DepthMapView::setBackgroundColor(QColor const& color)
{
  QTE_D();
  d->renderer->SetBackground(color.redF(), color.greenF(), color.blueF());
  d->UI.renderWidget->update();
}


//-----------------------------------------------------------------------------
void DepthMapView::setActiveFrame(unsigned frame)
{
  QTE_D();

//  d->featureRep->SetActiveFrame(frame);
//  d->UI.renderWidget->update();
}

//-----------------------------------------------------------------------------
void DepthMapView::resetView()
{
  QTE_D();

//  double renderAspect[2];
//  d->renderer->GetAspect(renderAspect);

//  auto const w = d->imageBounds[1] - d->imageBounds[0];
//  auto const h = d->imageBounds[3] - d->imageBounds[2];
//  auto const a = w / h;

//  auto const s = 0.5 * h * qMax(a / renderAspect[0], 1.0);

//  d->renderer->ResetCamera(d->imageBounds);
//  d->renderer->GetActiveCamera()->SetParallelScale(s);

//  d->UI.renderWidget->update();
}

//-----------------------------------------------------------------------------
void DepthMapView::resetViewToFullExtents()
{
  QTE_D();

//  d->renderer->ResetCamera();
//  d->UI.renderWidget->update();
}


//END DepthMapView
