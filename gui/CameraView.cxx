/*ckwg +29
 * Copyright 2017-2018 by Kitware, Inc.
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

#include "CameraView.h"

#include "ui_CameraView.h"
#include "am_CameraView.h"

#include "ActorColorButton.h"
#include "DataArrays.h"
#include "FeatureOptions.h"
#include "FieldInformation.h"
#include "GroundControlPointsWidget.h"
#include "ImageOptions.h"
#include "RulerWidget.h"
#include "vtkMaptkFeatureTrackRepresentation.h"

#include <vital/types/feature_track_set.h>
#include <vital/types/landmark_map.h>
#include <vital/types/track.h>

#include <vtkCamera.h>
#include <vtkCellArray.h>
#include <vtkDoubleArray.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkImageActor.h>
#include <vtkImageData.h>
#include <vtkInteractorStyleRubberBand2D.h>
#include <vtkMatrix4x4.h>
#include <vtkNew.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkUnsignedCharArray.h>
#include <vtkUnsignedIntArray.h>

#include <qtMath.h>
#include <qtUiState.h>

#include <QCheckBox>
#include <QFormLayout>
#include <QMenu>
#include <QTimer>
#include <QToolButton>
#include <QWidgetAction>

QTE_IMPLEMENT_D_FUNC(CameraView)

///////////////////////////////////////////////////////////////////////////////

//BEGIN miscelaneous helpers

using namespace LandmarkArrays;

namespace // anonymous
{

//-----------------------------------------------------------------------------
struct LandmarkData
{
  kwiver::vital::rgb_color color;
  double elevation;
  unsigned observations;
};

//-----------------------------------------------------------------------------
class ResidualsOptions : public QWidget
{
public:
  ResidualsOptions(QString const& settingsGroup, QWidget* parent);
  ~ResidualsOptions() override;

  void setDefaultInlierColor(QColor const&);
  void setDefaultOutlierColor(QColor const&);

  ActorColorButton* const inlierColorButton;
  ActorColorButton* const outlierColorButton;
  QCheckBox* const inlierCheckbox;
  qtUiState uiState;
};

//-----------------------------------------------------------------------------
ResidualsOptions::ResidualsOptions(
  QString const& settingsGroup, QWidget* parent)
  : QWidget(parent),
    inlierColorButton(new ActorColorButton(this)),
    outlierColorButton(new ActorColorButton(this)),
    inlierCheckbox(new QCheckBox(this))
{
  auto const layout = new QFormLayout(this);
  layout->addRow("Inlier Color", this->inlierColorButton);
  layout->addRow("Outlier Color", this->outlierColorButton);
  layout->addRow("Inliers Only", this->inlierCheckbox);

  this->inlierColorButton->persist(this->uiState,
                                   settingsGroup + "/InlierColor");
  this->outlierColorButton->persist(this->uiState,
                                    settingsGroup + "/OutlierColor");
  this->uiState.mapChecked(settingsGroup + "/Inlier", this->inlierCheckbox);
  this->uiState.restore();
}

//-----------------------------------------------------------------------------
ResidualsOptions::~ResidualsOptions()
{
  this->uiState.save();
}

//-----------------------------------------------------------------------------
void ResidualsOptions::setDefaultInlierColor(QColor const& color)
{
  this->inlierColorButton->setColor(color);
  this->uiState.restore();
}

//-----------------------------------------------------------------------------
void ResidualsOptions::setDefaultOutlierColor(QColor const& color)
{
  this->outlierColorButton->setColor(color);
  this->uiState.restore();
}

} // namespace <anonymous>

//END miscelaneous helpers

///////////////////////////////////////////////////////////////////////////////

//BEGIN CameraViewPrivate definition

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
    vtkNew<vtkPolyDataMapper> mapper;
    vtkNew<vtkActor> actor;
  };

  struct PointCloud : VertexCloud
  {
    PointCloud();

    void addPoint(double x, double y, double z);
  };

  struct SegmentCloud : VertexCloud
  {
    SegmentCloud();

    void addSegment(double x1, double y1, double z1,
                    double x2, double y2, double z2);
  };

  struct LandmarkCloud : PointCloud
  {
    LandmarkCloud();

    void addPoint(double x, double y, double z, LandmarkData const& data);

    void clear();

    vtkNew<vtkUnsignedCharArray> colors;
    vtkNew<vtkUnsignedIntArray> observations;
    vtkNew<vtkDoubleArray> elevations;
  };

  CameraViewPrivate() : featuresDirty(false), renderQueued(false) {}

  void setPopup(QAction* action, QMenu* menu);
  void setPopup(QAction* action, QWidget* widget);

  void setTransforms(int imageHeight);

  void updateFeatures(CameraView* q);

  Ui::CameraView UI;
  Am::CameraView AM;

  vtkNew<vtkRenderer> renderer;
  vtkSmartPointer<vtkRenderWindow> renderWindow;

  vtkNew<vtkImageActor> imageActor;
  vtkNew<vtkImageData> emptyImage;

  vtkNew<vtkMaptkFeatureTrackRepresentation> featureRep;

  vtkNew<vtkMatrix4x4> transformMatrix;

  LandmarkCloud landmarks;
  SegmentCloud residualsInlier;
  SegmentCloud residualsOutlier;

  QHash<kwiver::vital::landmark_id_t, LandmarkData> landmarkData;

  PointOptions* landmarkOptions;
  ResidualsOptions* residualsOptions;
  GroundControlPointsWidget* groundControlPointsWidget;
  RulerWidget* rulerWidget;

  double imageBounds[6];

  bool featuresDirty;

  bool renderQueued;
};

//END CameraViewPrivate definition

///////////////////////////////////////////////////////////////////////////////

//BEGIN geometry helpers

//-----------------------------------------------------------------------------
CameraViewPrivate::VertexCloud::VertexCloud()
{
  this->data->SetPoints(this->points);
  this->mapper->SetInputData(this->data);

  this->actor->SetMapper(this->mapper);
  this->actor->GetProperty()->SetPointSize(2);
}

//-----------------------------------------------------------------------------
void CameraViewPrivate::VertexCloud::clear()
{
  this->verts->Reset();
  this->points->Reset();

  this->points->Modified();
  this->verts->Modified();
}

//-----------------------------------------------------------------------------
CameraViewPrivate::PointCloud::PointCloud()
{
  this->data->SetVerts(this->verts);
}

//-----------------------------------------------------------------------------
void CameraViewPrivate::PointCloud::addPoint(double x, double y, double z)
{
  auto const vid = this->points->InsertNextPoint(x, y, z);

  this->verts->InsertNextCell(1);
  this->verts->InsertCellPoint(vid);

  this->points->Modified();
  this->verts->Modified();
}

//-----------------------------------------------------------------------------
CameraViewPrivate::SegmentCloud::SegmentCloud()
{
  this->data->SetVerts(this->verts);
  this->data->SetLines(this->verts);
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

  this->points->Modified();
  this->verts->Modified();
}

//-----------------------------------------------------------------------------
CameraViewPrivate::LandmarkCloud::LandmarkCloud()
{
  this->colors->SetName(TrueColor);
  this->colors->SetNumberOfComponents(3);

  this->observations->SetName(Observations);
  this->observations->SetNumberOfComponents(1);

  this->elevations->SetName(Elevation);
  this->elevations->SetNumberOfComponents(1);

  this->data->GetPointData()->AddArray(this->colors);
  this->data->GetPointData()->AddArray(this->observations);
  this->data->GetPointData()->AddArray(this->elevations);
}

//-----------------------------------------------------------------------------
void CameraViewPrivate::LandmarkCloud::clear()
{
  this->VertexCloud::clear();

  this->colors->Reset();
  this->observations->Reset();
  this->elevations->Reset();

  this->colors->Modified();
  this->observations->Modified();
  this->elevations->Modified();
}

//-----------------------------------------------------------------------------
void CameraViewPrivate::LandmarkCloud::addPoint(
  double x, double y, double z, LandmarkData const& data)
{
  this->PointCloud::addPoint(x, y, z);

  this->colors->InsertNextValue(data.color.r);
  this->colors->InsertNextValue(data.color.g);
  this->colors->InsertNextValue(data.color.b);

  this->observations->InsertNextValue(data.observations);

  this->elevations->InsertNextValue(data.elevation);

  this->colors->Modified();
  this->observations->Modified();
  this->elevations->Modified();
}

//END geometry helpers

///////////////////////////////////////////////////////////////////////////////

//BEGIN CameraViewPrivate implementation

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
  vtkMatrix4x4* xf = this->transformMatrix;
  xf->Identity();
  xf->SetElement(1, 1, -1.0);
  xf->SetElement(1, 3, imageHeight);

  this->featureRep->GetActivePointsWithDescActor()->SetUserMatrix(xf);
  this->featureRep->GetActivePointsWithoutDescActor()->SetUserMatrix(xf);
  this->featureRep->GetTrailsWithDescActor()->SetUserMatrix(xf);
  this->featureRep->GetTrailsWithoutDescActor()->SetUserMatrix(xf);
  this->landmarks.actor->SetUserMatrix(xf);
  this->residualsInlier.actor->SetUserMatrix(xf);
  this->residualsOutlier.actor->SetUserMatrix(xf);
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

//END CameraViewPrivate implementation

///////////////////////////////////////////////////////////////////////////////

//BEGIN CameraView

//-----------------------------------------------------------------------------
CameraView::CameraView(QWidget* parent, Qt::WindowFlags flags)
  : QWidget(parent, flags), d_ptr(new CameraViewPrivate)
{
  QTE_D();

  // Set up UI
  d->UI.setupUi(this);
  d->AM.setupActions(d->UI, this);
  d->renderWindow =
    vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();

  auto const viewMenu = new QMenu(this);
  viewMenu->addAction(d->UI.actionViewReset);
  viewMenu->addAction(d->UI.actionViewResetFullExtents);
  d->setPopup(d->UI.actionViewReset, viewMenu);

  auto const imageOptions = new ImageOptions("CameraView/Image", this);
  imageOptions->addActor(d->imageActor);
  d->setPopup(d->UI.actionShowFrameImage, imageOptions);

  connect(imageOptions, &ImageOptions::modified,
          this, &CameraView::render);

  auto const featureOptions =
    new FeatureOptions{d->featureRep, "CameraView/FeaturePoints", this};

  d->setPopup(d->UI.actionShowFeatures, featureOptions);

  connect(featureOptions, &FeatureOptions::modified,
          this, &CameraView::render);

  d->landmarkOptions = new PointOptions("CameraView/Landmarks", this);
  d->landmarkOptions->setDefaultColor(Qt::magenta);
  d->landmarkOptions->addActor(d->landmarks.actor);
  d->landmarkOptions->addMapper(d->landmarks.mapper);

  d->setPopup(d->UI.actionShowLandmarks, d->landmarkOptions);

  connect(d->landmarkOptions, &PointOptions::modified,
          this, &CameraView::render);

  d->groundControlPointsWidget = new GroundControlPointsWidget(this);
  d->groundControlPointsWidget->setTransformMatrix(d->transformMatrix);

  d->rulerWidget = new RulerWidget(this);
  d->rulerWidget->setTransformMatrix(d->transformMatrix);
  d->rulerWidget->setComputeDistance(false);

  d->residualsOptions =
    new ResidualsOptions("CameraView/Residuals", this);
  d->residualsOptions->setDefaultInlierColor(QColor(255, 128, 0));
  d->residualsOptions->setDefaultOutlierColor(QColor(0, 128, 255));
  d->residualsOptions->inlierColorButton->addActor(d->residualsInlier.actor);
  d->residualsOptions->outlierColorButton->addActor(d->residualsOutlier.actor);

  d->setPopup(d->UI.actionShowResiduals, d->residualsOptions);
  this->setOutlierResidualsVisible(
    d->residualsOptions->inlierCheckbox->isChecked());

  connect(d->residualsOptions->inlierColorButton,
          &ActorColorButton::colorChanged,
          this, &CameraView::render);
  connect(d->residualsOptions->outlierColorButton,
          &ActorColorButton::colorChanged,
          this, &CameraView::render);
  connect(d->residualsOptions->inlierCheckbox, &QCheckBox::toggled,
          this, &CameraView::setOutlierResidualsVisible);

  // Connect actions
  this->addAction(d->UI.actionViewReset);
  this->addAction(d->UI.actionViewResetFullExtents);
  this->addAction(d->UI.actionShowFeatures);
  this->addAction(d->UI.actionShowLandmarks);
  this->addAction(d->UI.actionShowResiduals);

  connect(d->UI.actionViewReset, &QAction::triggered,
          this, &CameraView::resetView);
  connect(d->UI.actionViewResetFullExtents, &QAction::triggered,
          this, &CameraView::resetViewToFullExtents);

  connect(d->UI.actionShowFrameImage, &QAction::toggled,
          this, &CameraView::setImageVisible);
  connect(d->UI.actionShowFeatures, &QAction::toggled,
          featureOptions, &FeatureOptions::setFeaturesWithDescVisible);
  connect(d->UI.actionShowFeatures, &QAction::toggled,
          featureOptions, &FeatureOptions::setFeaturesWithoutDescVisible);
  connect(d->UI.actionShowLandmarks, &QAction::toggled,
          this, &CameraView::setLandmarksVisible);
  connect(d->UI.actionShowResiduals, &QAction::toggled,
          this, &CameraView::setResidualsVisible);

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

  // Set interactor
#if VTK_VERSION_MAJOR < 9
  auto renderInteractor = d->UI.renderWidget->GetInteractor();
#else
  auto renderInteractor = d->UI.renderWidget->interactor();
#endif
  vtkNew<vtkInteractorStyleRubberBand2D> is;
  renderInteractor->SetInteractorStyle(is);
  d->groundControlPointsWidget->setInteractor(renderInteractor);
  d->rulerWidget->setInteractor(renderInteractor);

  // Set up actors
  d->renderer->AddActor(d->featureRep->GetActivePointsWithDescActor());
  d->renderer->AddActor(d->featureRep->GetActivePointsWithoutDescActor());
  d->renderer->AddActor(d->featureRep->GetTrailsWithDescActor());
  d->renderer->AddActor(d->featureRep->GetTrailsWithoutDescActor());
  d->renderer->AddActor(d->landmarks.actor);
  d->renderer->AddActor(d->residualsInlier.actor);
  d->renderer->AddActor(d->residualsOutlier.actor);

  d->renderer->AddViewProp(d->imageActor);
  d->imageActor->SetPosition(0.0, 0.0, -0.5);

  // Enable antialising by default
  d->renderer->UseFXAAOn();

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
void CameraView::setBackgroundColor(QColor const& color)
{
  QTE_D();
  d->renderer->SetBackground(color.redF(), color.greenF(), color.blueF());
  this->render();
}

//-----------------------------------------------------------------------------
void CameraView::setImagePath(QString const& path)
{
  QTE_D();
  d->UI.labelImagePath->setText(path);
}

//-----------------------------------------------------------------------------
void CameraView::setImageData(vtkImageData* data, QSize dimensions)
{
  QTE_D();

  if (!data)
  {
    // If no image given, clear current image and replace with "empty" image
    d->imageActor->SetInputData(d->emptyImage);

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
    auto const h = d->imageBounds[3] + 1 - d->imageBounds[2];
    d->setTransforms(qMax(0, static_cast<int>(h)));
  }

  this->render();
}

//-----------------------------------------------------------------------------
void CameraView::setActiveFrame(unsigned frame)
{
  QTE_D();

  d->featureRep->SetActiveFrame(frame);
  this->render();
}

//-----------------------------------------------------------------------------
void CameraView::setLandmarksData(kwiver::vital::landmark_map const& lm)
{
  QTE_D();

  auto const& landmarks = lm.landmarks();

  auto const defaultColor = kwiver::vital::rgb_color{};
  auto haveColor = false;
  auto maxObservations = unsigned{0};
  auto minZ = qInf(), maxZ = -qInf();
  auto autoMinZ = qInf(), autoMaxZ = -qInf();
  std::vector<double> zValues;

  foreach (auto const& lmi, landmarks)
  {
    auto const z = lmi.second->loc()[2];
    auto const& color = lmi.second->color();
    auto const observations = lmi.second->observations();
    auto const ld = LandmarkData{color, z, observations};

    d->landmarkData.insert(lmi.first, ld);

    haveColor = haveColor || (color != defaultColor);
    maxObservations = qMax(maxObservations, observations);
    minZ = qMin(minZ, z);
    maxZ = qMax(maxZ, z);
    zValues.push_back(z);
  }
  if (!zValues.empty())
  {
    // Set the range to cover the middle 90% of the data
    std::sort(zValues.begin(), zValues.end());
    auto const n = zValues.size();
    // index at 5% of the data
    auto const i = n / 20;
    autoMinZ = zValues[i];
    autoMaxZ = zValues[n - 1 - i];
  }

  auto fields = QHash<QString, FieldInformation>{};
  fields.insert("Elevation", FieldInformation{Elevation, {minZ, maxZ},
                                              {autoMinZ, autoMaxZ}});
  if (maxObservations)
  {
    auto const upper = static_cast<double>(maxObservations);
    fields.insert("Observations", FieldInformation{Observations, {0.0, upper},
                                                   {0.0, upper}});
  }

  d->landmarkOptions->setTrueColorAvailable(haveColor);
  d->landmarkOptions->setDataFields(fields);
}

//-----------------------------------------------------------------------------
void CameraView::addFeatureTrack(kwiver::vital::track const& track)
{
  QTE_D();

  auto const id = track.id();

  foreach (auto const& state, track)
  {
    auto const& fts =
      std::dynamic_pointer_cast<kwiver::vital::feature_track_state>(state);
    if ( !fts )
    {
      continue;
    }
    auto const& loc = fts->feature->loc();
    if (fts->descriptor)
    {
      d->featureRep->AddTrackWithDescPoint(
        id, state->frame(), loc[0], loc[1]);
    }
    else
    {
      d->featureRep->AddTrackWithoutDescPoint(
        id, state->frame(), loc[0], loc[1]);
    }
  }

  d->updateFeatures(this);
}

//-----------------------------------------------------------------------------
void CameraView::addLandmark(
  kwiver::vital::landmark_id_t id, double x, double y)
{
  QTE_D();

  d->landmarks.addPoint(x, y, 0.0, d->landmarkData.value(id));
}

//-----------------------------------------------------------------------------
void CameraView::addResidual(
  kwiver::vital::track_id_t id, double x1, double y1, double x2, double y2,
  bool inlier)
{
  QTE_D();

  Q_UNUSED(id)

  if (inlier)
  {
    d->residualsInlier.addSegment(x1, y1, -0.3, x2, y2, -0.3);
  }
  else
  {
    d->residualsOutlier.addSegment(x1, y1, -0.2, x2, y2, -0.2);
  }
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
  d->residualsInlier.clear();
  d->residualsOutlier.clear();
}

//-----------------------------------------------------------------------------
void CameraView::clearFeatureTracks()
{
  QTE_D();
  d->featureRep->ClearTrackData();
  d->updateFeatures(this);
}

//-----------------------------------------------------------------------------
void CameraView::setImageVisible(bool state)
{
  QTE_D();

  d->imageActor->SetVisibility(state);
  this->render();
}

//-----------------------------------------------------------------------------
void CameraView::setLandmarksVisible(bool state)
{
  QTE_D();

  d->landmarks.actor->SetVisibility(state);
  this->render();
}

//-----------------------------------------------------------------------------
void CameraView::setResidualsVisible(bool state)
{
  QTE_D();

  d->residualsInlier.actor->SetVisibility(state);
  d->residualsOutlier.actor->SetVisibility(state &&
    !d->residualsOptions->inlierCheckbox->isChecked());
  this->render();
}

//-----------------------------------------------------------------------------
void CameraView::setOutlierResidualsVisible(bool state)
{
  QTE_D();

  bool overallState = d->residualsInlier.actor->GetVisibility();
  d->residualsOutlier.actor->SetVisibility(overallState && !state);
  this->render();
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

  this->render();
}

//-----------------------------------------------------------------------------
void CameraView::resetViewToFullExtents()
{
  QTE_D();

  d->renderer->ResetCamera();
  this->render();
}

//-----------------------------------------------------------------------------
void CameraView::updateFeatures()
{
  QTE_D();

  if (d->featuresDirty)
  {
    d->featureRep->Update();
    this->render();

    d->featuresDirty = false;
  }
}

//-----------------------------------------------------------------------------
GroundControlPointsWidget* CameraView::groundControlPointsWidget() const
{
  QTE_D();

  return d->groundControlPointsWidget;
}

//-----------------------------------------------------------------------------
RulerWidget* CameraView::rulerWidget() const
{
  QTE_D();

  return d->rulerWidget;
}

//-----------------------------------------------------------------------------
void CameraView::render()
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
void CameraView::enableAntiAliasing(bool enable)
{
  QTE_D();

  d->renderer->SetUseFXAA(enable);
  this->render();
}

//-----------------------------------------------------------------------------
void CameraView::clearGroundControlPoints()
{
  QTE_D();

  d->groundControlPointsWidget->clearPoints();
  this->render();
}

//END CameraView
