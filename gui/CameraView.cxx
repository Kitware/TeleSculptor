/*ckwg +29
 * Copyright 2015 by Kitware, Inc.
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

#include "CameraView.h"

#include "ui_CameraView.h"
#include "am_CameraView.h"

#include "ActorColorButton.h"
#include "FeatureOptions.h"
#include "ImageOptions.h"
#include "vtkMaptkCamera.h"
#include "vtkMaptkFeatureTrackRepresentation.h"

#include <maptk/track.h>

#include <vtkCamera.h>
#include <vtkCellArray.h>
#include <vtkImageActor.h>
#include <vtkImageData.h>
#include <vtkInteractorStyleRubberBand2D.h>
#include <vtkMatrix4x4.h>
#include <vtkNew.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>

#include <qtUiState.h>

#include <QtGui/QFormLayout>
#include <QtGui/QMenu>
#include <QtGui/QToolButton>
#include <QtGui/QWidgetAction>

#include <QtCore/QDebug>

namespace // anonymous
{

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

//-----------------------------------------------------------------------------
class CameraViewPrivate
{
public:
  struct VertexCloud
  {
    VertexCloud();

    void clear();

    vtkNew<vtkPoints> points;
    vtkNew<vtkCellArray> verts;
    vtkNew<vtkPolyData> data;
    vtkNew<vtkActor> actor;
  };

  struct PointCloud : VertexCloud
  {
    PointCloud();

    void addPoint(double x, double y, double z);
  };

  struct SegmentCloud : PointCloud
  {
    SegmentCloud();

    void addSegment(double x1, double y1, double z1,
                    double x2, double y2, double z2);
  };

  CameraViewPrivate() : featuresDirty(false) {}

  void setPopup(QAction* action, QMenu* menu);
  void setPopup(QAction* action, QWidget* widget);

  void setTransforms(int imageHeight);

  void updateFeatures(CameraView* q);

  Ui::CameraView UI;
  Am::CameraView AM;

  vtkNew<vtkRenderer> renderer;
  vtkNew<vtkRenderWindow> renderWindow;

  vtkNew<vtkImageActor> imageActor;
  vtkNew<vtkImageData> emptyImage;

  vtkNew<vtkMaptkFeatureTrackRepresentation> featureRep;

  PointCloud landmarks;
  SegmentCloud residuals;

  double imageBounds[6];

  bool featuresDirty;
};

QTE_IMPLEMENT_D_FUNC(CameraView)

//-----------------------------------------------------------------------------
CameraViewPrivate::VertexCloud::VertexCloud()
{
  vtkNew<vtkPolyDataMapper> mapper;

  this->data->SetPoints(this->points.GetPointer());
  mapper->SetInputData(this->data.GetPointer());

  this->actor->SetMapper(mapper.GetPointer());
  this->actor->GetProperty()->SetPointSize(2);
}

//-----------------------------------------------------------------------------
void CameraViewPrivate::VertexCloud::clear()
{
  this->verts->Reset();
  this->points->Reset();
}

//-----------------------------------------------------------------------------
CameraViewPrivate::PointCloud::PointCloud()
{
  this->data->SetVerts(this->verts.GetPointer());
}

//-----------------------------------------------------------------------------
void CameraViewPrivate::PointCloud::addPoint(double x, double y, double z)
{
  auto const vid = this->points->InsertNextPoint(x, y, z);

  this->verts->InsertNextCell(1);
  this->verts->InsertCellPoint(vid);
}

//-----------------------------------------------------------------------------
CameraViewPrivate::SegmentCloud::SegmentCloud()
{
  this->data->SetLines(this->verts.GetPointer());
}

//-----------------------------------------------------------------------------
void CameraViewPrivate::SegmentCloud::addSegment(
  double x1, double y1, double z1,
  double x2, double y2, double z2)
{
  auto const vid1 = this->points->InsertNextPoint(x1, y1, z1);
  auto const vid2 = this->points->InsertNextPoint(x2, y2, z2);

  this->verts->InsertNextCell(2);
  this->verts->InsertCellPoint(vid1);
  this->verts->InsertCellPoint(vid2);
}

//-----------------------------------------------------------------------------
void CameraViewPrivate::setPopup(QAction* action, QMenu* menu)
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
void CameraViewPrivate::setPopup(QAction* action, QWidget* widget)
{
  auto const parent = action->parentWidget();

  auto const proxy = new QWidgetAction(parent);
  proxy->setDefaultWidget(widget);

  auto const menu = new QMenu(parent);
  menu->addAction(proxy);

  this->setPopup(action, menu);
}

//-----------------------------------------------------------------------------
void CameraViewPrivate::setTransforms(int imageHeight)
{
  vtkNew<vtkMatrix4x4> xf;

  xf->Identity();
  xf->SetElement(1, 1, -1.0);
  xf->SetElement(1, 3, imageHeight);

  this->featureRep->GetActivePointsActor()->SetUserMatrix(xf.GetPointer());
  this->featureRep->GetTrailsActor()->SetUserMatrix(xf.GetPointer());
  this->landmarks.actor->SetUserMatrix(xf.GetPointer());
  this->residuals.actor->SetUserMatrix(xf.GetPointer());
}

//-----------------------------------------------------------------------------
void CameraViewPrivate::updateFeatures(CameraView* q)
{
  if (!this->featuresDirty)
  {
    this->featuresDirty = true;
    QMetaObject::invokeMethod(q, "updateFeatures", Qt::QueuedConnection);
  }
}

//-----------------------------------------------------------------------------
CameraView::CameraView(QWidget* parent, Qt::WindowFlags flags)
  : QWidget(parent, flags), d_ptr(new CameraViewPrivate)
{
  QTE_D();

  // Set up UI
  d->UI.setupUi(this);
  d->AM.setupActions(d->UI, this);

  this->addAction(d->UI.actionViewReset);
  this->addAction(d->UI.actionViewResetFullExtents);

  auto const viewMenu = new QMenu(this);
  viewMenu->addAction(d->UI.actionViewReset);
  viewMenu->addAction(d->UI.actionViewResetFullExtents);
  d->setPopup(d->UI.actionViewReset, viewMenu);

  auto const imageOptions = new ImageOptions("CameraView/Image", this);
  imageOptions->addActor(d->imageActor.GetPointer());
  d->setPopup(d->UI.actionShowFrameImage, imageOptions);

  connect(imageOptions, SIGNAL(modified()),
          d->UI.renderWidget, SLOT(update()));

  auto const featureOptions =
    new FeatureOptions(d->featureRep.GetPointer(),
                       "CameraView/FeaturePoints", this);

  d->setPopup(d->UI.actionShowFeatures, featureOptions);

  connect(featureOptions, SIGNAL(modified()),
          d->UI.renderWidget, SLOT(update()));

  auto const landmarkOptions =
    new PointOptions("CameraView/Landmarks", this);
  landmarkOptions->setDefaultColor(Qt::magenta);
  landmarkOptions->addActor(d->landmarks.actor.GetPointer());

  d->setPopup(d->UI.actionShowLandmarks, landmarkOptions);

  connect(landmarkOptions, SIGNAL(modified()),
          d->UI.renderWidget, SLOT(update()));

  auto const residualsOptions =
    new ActorColorOption("CameraView/Residuals", this);
  residualsOptions->setDefaultColor(QColor(255, 128, 0));
  residualsOptions->button->addActor(d->residuals.actor.GetPointer());

  d->setPopup(d->UI.actionShowResiduals, residualsOptions);

  connect(residualsOptions->button, SIGNAL(colorChanged(QColor)),
          d->UI.renderWidget, SLOT(update()));

  // Connect actions
  this->addAction(d->UI.actionViewReset);
  this->addAction(d->UI.actionViewResetFullExtents);
  this->addAction(d->UI.actionShowFeatures);
  this->addAction(d->UI.actionShowLandmarks);
  this->addAction(d->UI.actionShowResiduals);

  connect(d->UI.actionViewReset, SIGNAL(triggered()),
          this, SLOT(resetView()));
  connect(d->UI.actionViewResetFullExtents, SIGNAL(triggered()),
          this, SLOT(resetViewToFullExtents()));

  connect(d->UI.actionShowFrameImage, SIGNAL(toggled(bool)),
          this, SLOT(setImageVisible(bool)));
  connect(d->UI.actionShowFeatures, SIGNAL(toggled(bool)),
          featureOptions, SLOT(setFeaturesVisible(bool)));
  connect(d->UI.actionShowLandmarks, SIGNAL(toggled(bool)),
          this, SLOT(setLandmarksVisible(bool)));
  connect(d->UI.actionShowResiduals, SIGNAL(toggled(bool)),
          this, SLOT(setResidualsVisible(bool)));

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

  // Set up actors
  d->renderer->AddActor(d->featureRep->GetActivePointsActor());
  d->renderer->AddActor(d->featureRep->GetTrailsActor());
  d->renderer->AddActor(d->landmarks.actor.GetPointer());
  d->renderer->AddActor(d->residuals.actor.GetPointer());

  d->renderer->AddViewProp(d->imageActor.GetPointer());
  d->imageActor->SetPosition(0.0, 0.0, -0.5);

  // Create "dummy" image data for use when we have no "real" image
  d->emptyImage->SetExtent(0, 0, 0, 0, 0, 0);
  d->emptyImage->AllocateScalars(VTK_UNSIGNED_CHAR, 1);
  d->emptyImage->SetScalarComponentFromDouble(0, 0, 0, 0, 0.0);

  this->setImageData(0, QSize(1, 1));
}

//-----------------------------------------------------------------------------
CameraView::~CameraView()
{
}

//-----------------------------------------------------------------------------
void CameraView::setImageData(vtkImageData* data, QSize const& dimensions)
{
  QTE_D();

  if (!data)
  {
    // If no image given, clear current image and replace with "empty" image
    d->imageActor->SetInputData(d->emptyImage.GetPointer());

    d->imageBounds[0] = 0.0; d->imageBounds[1] = dimensions.width() - 1;
    d->imageBounds[2] = 0.0; d->imageBounds[3] = dimensions.height() - 1;
    d->imageBounds[4] = 0.0; d->imageBounds[5] = 0.0;

    d->setTransforms(dimensions.height());
  }
  else
  {
    // Set data on image actor
    d->imageActor->SetInputData(data);
    d->imageActor->Update();

    d->imageActor->GetBounds(d->imageBounds);
    auto const w = d->imageBounds[1] + 1 - d->imageBounds[0];
    auto const h = d->imageBounds[3] + 1 - d->imageBounds[2];
    d->setTransforms(qMax(0, static_cast<int>(h)));
  }

  d->UI.renderWidget->update();
}

//-----------------------------------------------------------------------------
void CameraView::setActiveFrame(unsigned frame)
{
  QTE_D();

  d->featureRep->SetActiveFrame(frame);
  this->update();
}

//-----------------------------------------------------------------------------
void CameraView::addFeatureTrack(maptk::track const& track)
{
  QTE_D();

  auto const id = track.id();
  auto const end = track.end();

  for (auto iter = track.begin(); iter != end; ++iter)
  {
    auto const& loc = iter->feat->loc();
    d->featureRep->AddTrackPoint(id, iter->frame_id, loc[0], loc[1]);
  }

  d->updateFeatures(this);
}

//-----------------------------------------------------------------------------
void CameraView::addLandmark(unsigned int id, double x, double y)
{
  QTE_D();

  Q_UNUSED(id)

  d->landmarks.addPoint(x, y, 0.0);
  this->update();
}

//-----------------------------------------------------------------------------
void CameraView::addResidual(
  unsigned int id, double x1, double y1, double x2, double y2)
{
  QTE_D();

  Q_UNUSED(id)

  d->residuals.addSegment(x1, y1, -0.2, x2, y2, -0.2);

  this->update();
}

//-----------------------------------------------------------------------------
void CameraView::clearLandmarks()
{
  QTE_D();
  d->landmarks.clear();
}

//-----------------------------------------------------------------------------
void CameraView::clearResiduals()
{
  QTE_D();
  d->residuals.clear();
}

//-----------------------------------------------------------------------------
void CameraView::setImageVisible(bool state)
{
  QTE_D();

  d->imageActor->SetVisibility(state);
  d->UI.renderWidget->update();
}

//-----------------------------------------------------------------------------
void CameraView::setLandmarksVisible(bool state)
{
  QTE_D();

  d->landmarks.actor->SetVisibility(state);
  d->UI.renderWidget->update();
}

//-----------------------------------------------------------------------------
void CameraView::setResidualsVisible(bool state)
{
  QTE_D();

  d->residuals.actor->SetVisibility(state);
  d->UI.renderWidget->update();
}

//-----------------------------------------------------------------------------
void CameraView::resetView()
{
  QTE_D();

  double renderAspect[2];
  d->renderer->GetAspect(renderAspect);

  auto const w = d->imageBounds[1] - d->imageBounds[0];
  auto const h = d->imageBounds[3] - d->imageBounds[2];
  auto const a = w / h;

  auto const s = 0.5 * h * qMax(a / renderAspect[0], 1.0);

  d->renderer->ResetCamera(d->imageBounds);
  d->renderer->GetActiveCamera()->SetParallelScale(s);

  d->UI.renderWidget->update();
}

//-----------------------------------------------------------------------------
void CameraView::resetViewToFullExtents()
{
  QTE_D();

  d->renderer->ResetCamera();
  d->UI.renderWidget->update();
}

//-----------------------------------------------------------------------------
void CameraView::updateFeatures()
{
  QTE_D();

  if (d->featuresDirty)
  {
    d->featureRep->Update();
    this->update();

    d->featuresDirty = false;
  }
}
