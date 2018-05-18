/*ckwg +29
 * Copyright 2016-2017 by Kitware, Inc.
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

#include "WorldView.h"

#include "ui_WorldView.h"
#include "am_WorldView.h"

#include "CameraOptions.h"
#include "DataArrays.h"
#include "DepthMapOptions.h"
#include "FieldInformation.h"
#include "ImageOptions.h"
#include "PointOptions.h"
#include "VolumeOptions.h"
#include "vtkMaptkImageUnprojectDepth.h"
#include "vtkMaptkCamera.h"
#include "vtkMaptkCameraRepresentation.h"
#include "vtkMaptkScalarDataFilter.h"

#include <vital/types/camera.h>
#include <vital/types/landmark_map.h>

#include <vtkBoundingBox.h>
#include <vtkCellArray.h>
#include <vtkCellDataToPointData.h>
#include <vtkContourFilter.h>
#include <vtkCubeAxesActor.h>
#include <vtkDoubleArray.h>
#include <vtkGeometryFilter.h>
#include <vtkImageActor.h>
#include <vtkImageData.h>
#include <vtkMaptkImageDataGeometryFilter.h>
#include <vtkMatrix4x4.h>
#include <vtkNew.h>
#include <vtkPlaneSource.h>
#include <vtkPLYWriter.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkStructuredGrid.h>
#include <vtkTextProperty.h>
#include <vtkThreshold.h>
#include <vtkTimeStamp.h>
#include <vtkUnsignedCharArray.h>
#include <vtkUnsignedIntArray.h>
#include <vtkXMLImageDataReader.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkXMLStructuredGridReader.h>
#include <vtkXMLStructuredGridWriter.h>


#ifdef VTKWEBGLEXPORTER
#include <vtkScalarsToColors.h>
#include <vtkWebGLExporter.h>
#endif

#include <qtIndexRange.h>
#include <qtMath.h>

#include <QDebug>
#include <QFileInfo>
#include <QMenu>
#include <QToolButton>
#include <QWidgetAction>

using namespace LandmarkArrays;

QTE_IMPLEMENT_D_FUNC(WorldView)

//-----------------------------------------------------------------------------
class WorldViewPrivate
{
public:
  WorldViewPrivate()
    : rangeUpdateNeeded(false),
      validDepthInput(false),
      validImage(false),
      validTransform(false),
      cameraRepDirty(false),
      scaleDirty(false),
      axesDirty(false),
      axesVisible(false)
  {
  }

  void setPopup(QAction* action, QMenu* menu);
  void setPopup(QAction* action, QWidget* widget);

  void setView(kwiver::vital::vector_3d const& normal,
               kwiver::vital::vector_3d const& up);
  void setView(double ni, double nj, double nk,
               double ui, double uj, double uk);

  void updateImageTransform();
  void updateCameras(WorldView*);
  void updateScale(WorldView*);
  void updateAxes(WorldView*, bool immediate = false);

  Ui::WorldView UI;
  Am::WorldView AM;

  vtkNew<vtkRenderer> renderer;
  vtkNew<vtkRenderWindow> renderWindow;

  vtkNew<vtkMaptkCameraRepresentation> cameraRep;

  vtkNew<vtkPoints> landmarkPoints;
  vtkNew<vtkCellArray> landmarkVerts;
  vtkNew<vtkDoubleArray> landmarkElevations;
  vtkNew<vtkUnsignedCharArray> landmarkColors;
  vtkNew<vtkUnsignedIntArray> landmarkObservations;
  vtkNew<vtkPolyDataMapper> landmarkMapper;
  vtkNew<vtkActor> landmarkActor;

  vtkNew<vtkImageActor> imageActor;
  vtkNew<vtkImageData> emptyImage;

  vtkNew<vtkPlaneSource> groundPlane;
  vtkNew<vtkActor> groundActor;

  vtkNew<vtkCubeAxesActor> cubeAxesActor;

  ImageOptions* imageOptions;
  CameraOptions* cameraOptions;
  PointOptions* landmarkOptions;
  DepthMapOptions* depthMapOptions;

  VolumeOptions* volumeOptions;
  vtkContourFilter* contourFilter;

  vtkNew<vtkMatrix4x4> imageProjection;
  vtkNew<vtkMatrix4x4> imageLocalTransform;

  vtkSmartPointer<vtkMaptkImageDataGeometryFilter> inputDepthGeometryFilter;
  vtkNew<vtkMaptkScalarDataFilter> depthScalarFilter;
  vtkNew<vtkActor> depthMapActor;

  vtkNew<vtkActor> volumeActor;
  vtkStructuredGrid* volume;

  bool rangeUpdateNeeded;
  bool validDepthInput;
  bool validImage;
  bool validTransform;

  bool cameraRepDirty;
  bool scaleDirty;
  bool axesDirty;

  bool axesVisible;
};

//-----------------------------------------------------------------------------
void WorldViewPrivate::setPopup(QAction* action, QMenu* menu)
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
void WorldViewPrivate::setPopup(QAction* action, QWidget* widget)
{
  auto const parent = action->parentWidget();

  auto const proxy = new QWidgetAction(parent);
  proxy->setDefaultWidget(widget);

  auto const menu = new QMenu(parent);
  menu->addAction(proxy);

  this->setPopup(action, menu);
}

//-----------------------------------------------------------------------------
void WorldViewPrivate::setView(
  double ni, double nj, double nk, double ui, double uj, double uk)
{
  this->setView(kwiver::vital::vector_3d(ni, nj, nk),
                kwiver::vital::vector_3d(ui, uj, uk));
}

//-----------------------------------------------------------------------------
void WorldViewPrivate::setView(
  kwiver::vital::vector_3d const& normal, kwiver::vital::vector_3d const& up)
{
  // Get camera
  auto const camera = this->renderer->GetActiveCamera();

  // Get camera parameters
  auto focus = kwiver::vital::vector_3d();
  auto pos = kwiver::vital::vector_3d();
  camera->GetPosition(pos.data());
  camera->GetFocalPoint(focus.data());

  // Compute new position
  pos = focus + (normal * (focus - pos).norm());

  // Modify camera
  camera->SetPosition(pos.data());
  camera->SetViewUp(up.data());

  this->UI.renderWidget->update();
}

//-----------------------------------------------------------------------------
void WorldViewPrivate::updateImageTransform()
{
  vtkNew<vtkMatrix4x4> xf;

  vtkMatrix4x4::Multiply4x4(
    this->imageProjection.GetPointer(),
    this->imageLocalTransform.GetPointer(),
    xf.GetPointer());

  this->imageActor->SetUserMatrix(xf.GetPointer());
}

//-----------------------------------------------------------------------------
void WorldViewPrivate::updateCameras(WorldView* q)
{
  if (!this->cameraRepDirty)
  {
    this->cameraRepDirty = true;
    QMetaObject::invokeMethod(q, "updateCameras", Qt::QueuedConnection);

    // If the cameras are dirty, then the camera scale must also be dirty
    this->updateScale(q);
  }
}

//-----------------------------------------------------------------------------
void WorldViewPrivate::updateScale(WorldView* q)
{
  if (!this->scaleDirty)
  {
    this->scaleDirty = true;
    QMetaObject::invokeMethod(q, "updateScale", Qt::QueuedConnection);
  }
}

//-----------------------------------------------------------------------------
void WorldViewPrivate::updateAxes(WorldView* q, bool immediate)
{
  if (immediate)
  {
    this->axesDirty = true;
    q->updateAxes();
  }
  else if (!this->axesDirty)
  {
    this->axesDirty = true;
    QMetaObject::invokeMethod(q, "updateAxes", Qt::QueuedConnection);
  }
}

//-----------------------------------------------------------------------------
WorldView::WorldView(QWidget* parent, Qt::WindowFlags flags)
  : QWidget(parent, flags), d_ptr(new WorldViewPrivate)
{
  QTE_D();

  // Set up UI
  d->UI.setupUi(this);
  d->AM.setupActions(d->UI, this);

  auto const viewMenu = new QMenu(this);
  viewMenu->addAction(d->UI.actionViewReset);
  viewMenu->addAction(d->UI.actionViewResetLandmarks);
  viewMenu->addSeparator();
  viewMenu->addAction(d->UI.actionViewToWorldTop);
  viewMenu->addAction(d->UI.actionViewToWorldLeft);
  viewMenu->addAction(d->UI.actionViewToWorldRight);
  viewMenu->addAction(d->UI.actionViewToWorldFront);
  viewMenu->addAction(d->UI.actionViewToWorldBack);
  viewMenu->addSeparator();
  viewMenu->addAction(d->UI.actionViewPerspective);
  d->setPopup(d->UI.actionViewReset, viewMenu);

  d->imageOptions = new ImageOptions("WorldView/Image", this);
  d->imageOptions->addActor(d->imageActor.GetPointer());
  d->setPopup(d->UI.actionShowFrameImage, d->imageOptions);

  connect(d->imageOptions, SIGNAL(modified()),
          d->UI.renderWidget, SLOT(update()));

  d->cameraOptions = new CameraOptions(d->cameraRep.GetPointer(), this);
  d->setPopup(d->UI.actionShowCameras, d->cameraOptions);

  connect(d->cameraOptions, SIGNAL(modified()),
          this, SLOT(invalidateGeometry()));

  d->landmarkOptions = new PointOptions("WorldView/Landmarks", this);
  d->landmarkOptions->addActor(d->landmarkActor.GetPointer());
  d->setPopup(d->UI.actionShowLandmarks, d->landmarkOptions);

  connect(d->landmarkOptions, SIGNAL(modified()),
          d->UI.renderWidget, SLOT(update()));

  d->depthMapOptions = new DepthMapOptions("WorldView/DepthMap", this);
  d->setPopup(d->UI.actionShowDepthMap, d->depthMapOptions);

  d->depthMapOptions->setEnabled(false);

  connect(d->UI.actionShowVolume, SIGNAL(triggered(bool)),
          this, SIGNAL(meshEnabled(bool)));
  connect(d->depthMapOptions, SIGNAL(displayModeChanged()),
          this, SLOT(updateDepthMapDisplayMode()));
  connect(d->depthMapOptions, SIGNAL(thresholdsChanged(bool)),
          this, SLOT(updateDepthMapThresholds(bool)));

  d->volumeOptions = new VolumeOptions("WorldView/Volume", this);
  d->setPopup(d->UI.actionShowVolume, d->volumeOptions);

  d->volumeOptions->setEnabled(false);

  connect(d->volumeOptions, SIGNAL(modified()),
          d->UI.renderWidget, SLOT(update()));

  connect(d->volumeOptions, SIGNAL(colorOptionsEnabled(bool)),
          this, SIGNAL(coloredMeshEnabled(bool)));

  connect(this, SIGNAL(contourChanged()),
          d->UI.renderWidget, SLOT(update()));

  // Connect actions
  this->addAction(d->UI.actionViewReset);
  this->addAction(d->UI.actionViewResetLandmarks);
  this->addAction(d->UI.actionViewPerspective);
  this->addAction(d->UI.actionShowCameras);
  this->addAction(d->UI.actionShowLandmarks);
  this->addAction(d->UI.actionShowGroundPlane);
  this->addAction(d->UI.actionShowDepthMap);
  this->addAction(d->UI.actionShowVolume);

  connect(d->UI.actionViewReset, SIGNAL(triggered()),
          this, SLOT(resetView()));
  connect(d->UI.actionViewResetLandmarks, SIGNAL(triggered()),
          this, SLOT(resetViewToLandmarks()));
  connect(d->UI.actionViewToWorldTop, SIGNAL(triggered()),
          this, SLOT(viewToWorldTop()));
  connect(d->UI.actionViewToWorldLeft, SIGNAL(triggered()),
          this, SLOT(viewToWorldLeft()));
  connect(d->UI.actionViewToWorldRight, SIGNAL(triggered()),
          this, SLOT(viewToWorldRight()));
  connect(d->UI.actionViewToWorldFront, SIGNAL(triggered()),
          this, SLOT(viewToWorldFront()));
  connect(d->UI.actionViewToWorldBack, SIGNAL(triggered()),
          this, SLOT(viewToWorldBack()));
  connect(d->UI.actionViewPerspective, SIGNAL(toggled(bool)),
          this, SLOT(setPerspective(bool)));

  connect(d->UI.actionShowFrameImage, SIGNAL(toggled(bool)),
          this, SLOT(setImageVisible(bool)));
  connect(d->UI.actionShowCameras, SIGNAL(toggled(bool)),
          this, SLOT(setCamerasVisible(bool)));
  connect(d->UI.actionShowLandmarks, SIGNAL(toggled(bool)),
          this, SLOT(setLandmarksVisible(bool)));
  connect(d->UI.actionShowGroundPlane, SIGNAL(toggled(bool)),
          this, SLOT(setGroundPlaneVisible(bool)));
  connect(d->UI.actionShowDepthMap, SIGNAL(toggled(bool)),
          this, SLOT(setDepthMapVisible(bool)));
  connect(d->UI.actionShowDepthMap, SIGNAL(toggled(bool)),
          this, SIGNAL(depthMapEnabled(bool)));

  connect(d->UI.actionShowVolume, SIGNAL(toggled(bool)),
          this, SLOT(setVolumeVisible(bool)));
  connect(d->UI.actionShowVolume, SIGNAL(toggled(bool)),
          this, SIGNAL(meshEnabled(bool)));
  connect(d->volumeOptions, SIGNAL(colorOptionsEnabled(bool)),
          this, SIGNAL(coloredMeshEnabled(bool)));

  // Set up render pipeline
  d->renderer->SetBackground(0, 0, 0);
  d->renderWindow->AddRenderer(d->renderer.GetPointer());
  d->UI.renderWidget->SetRenderWindow(d->renderWindow.GetPointer());

  d->renderer->AddActor(d->cameraRep->GetNonActiveActor());
  d->renderer->AddActor(d->cameraRep->GetActiveActor());
  d->renderer->AddActor(d->cameraRep->GetPathActor());

  // Set up image actor and "dummy" data for use when we have no "real" image
  d->imageActor->SetVisibility(false);
  d->renderer->AddViewProp(d->imageActor.GetPointer());

  d->emptyImage->SetExtent(0, 0, 0, 0, 0, 0);
  d->emptyImage->AllocateScalars(VTK_UNSIGNED_CHAR, 1);
  d->emptyImage->SetScalarComponentFromDouble(0, 0, 0, 0, 0.0);

  this->setImageData(0, QSize(1, 1));

  // Set up landmark actor
  vtkNew<vtkPolyData> landmarkPolyData;

  auto const landmarkPointData = landmarkPolyData->GetPointData();

  d->landmarkColors->SetName(TrueColor);
  d->landmarkColors->SetNumberOfComponents(3);

  d->landmarkElevations->SetName(Elevation);
  d->landmarkElevations->SetNumberOfComponents(1);

  d->landmarkObservations->SetName(Observations);
  d->landmarkObservations->SetNumberOfComponents(1);

  landmarkPolyData->SetPoints(d->landmarkPoints.GetPointer());
  landmarkPolyData->SetVerts(d->landmarkVerts.GetPointer());
  landmarkPointData->AddArray(d->landmarkColors.GetPointer());
  landmarkPointData->AddArray(d->landmarkElevations.GetPointer());
  landmarkPointData->AddArray(d->landmarkObservations.GetPointer());
  d->landmarkMapper->SetInputData(landmarkPolyData.GetPointer());

  d->landmarkActor->SetMapper(d->landmarkMapper.GetPointer());
  d->landmarkActor->SetVisibility(d->UI.actionShowLandmarks->isChecked());
  d->renderer->AddActor(d->landmarkActor.GetPointer());

  d->landmarkOptions->addMapper(d->landmarkMapper.GetPointer());

  // Set up ground plane grid
  d->groundPlane->SetOrigin(-10.0, -10.0, 0.0);
  d->groundPlane->SetPoint1(+10.0, -10.0, 0.0);
  d->groundPlane->SetPoint2(-10.0, +10.0, 0.0);
  d->groundPlane->SetResolution(20, 20);

  vtkNew<vtkPolyDataMapper> groundMapper;
  groundMapper->SetInputConnection(d->groundPlane->GetOutputPort());
  groundMapper->SetResolveCoincidentTopologyToPolygonOffset();

  d->groundActor->SetMapper(groundMapper.GetPointer());
  d->groundActor->GetProperty()->SetColor(0.5, 0.5, 0.5);
  d->groundActor->GetProperty()->SetLighting(false);
  d->groundActor->GetProperty()->SetRepresentationToWireframe();
  d->renderer->AddActor(d->groundActor.GetPointer());

  // Set up axes
  d->cubeAxesActor->GetTitleTextProperty(0)->SetColor(1.0, 0.3, 0.3);
  d->cubeAxesActor->GetLabelTextProperty(0)->SetColor(1.0, 0.3, 0.3);
  d->cubeAxesActor->GetTitleTextProperty(1)->SetColor(0.0, 1.0, 0.0);
  d->cubeAxesActor->GetLabelTextProperty(1)->SetColor(0.0, 1.0, 0.0);
  d->cubeAxesActor->GetTitleTextProperty(2)->SetColor(0.4, 0.4, 1.0);
  d->cubeAxesActor->GetLabelTextProperty(2)->SetColor(0.4, 0.4, 1.0);

  d->cubeAxesActor->GetXAxesLinesProperty()->SetColor(0.5, 0.5, 0.5);
  d->cubeAxesActor->GetYAxesLinesProperty()->SetColor(0.5, 0.5, 0.5);
  d->cubeAxesActor->GetZAxesLinesProperty()->SetColor(0.5, 0.5, 0.5);
  d->cubeAxesActor->GetXAxesGridlinesProperty()->SetColor(0.5, 0.5, 0.5);
  d->cubeAxesActor->GetYAxesGridlinesProperty()->SetColor(0.5, 0.5, 0.5);
  d->cubeAxesActor->GetZAxesGridlinesProperty()->SetColor(0.5, 0.5, 0.5);

  d->cubeAxesActor->DrawXGridlinesOn();
  d->cubeAxesActor->DrawYGridlinesOn();
  d->cubeAxesActor->DrawZGridlinesOn();
  d->cubeAxesActor->XAxisMinorTickVisibilityOff();
  d->cubeAxesActor->YAxisMinorTickVisibilityOff();
  d->cubeAxesActor->ZAxisMinorTickVisibilityOff();
  d->cubeAxesActor->SetFlyMode(vtkCubeAxesActor::VTK_FLY_FURTHEST_TRIAD);
  d->cubeAxesActor->SetGridLineLocation(vtkCubeAxesActor::VTK_GRID_LINES_FURTHEST);

  d->cubeAxesActor->SetCamera(d->renderer->GetActiveCamera());
  d->cubeAxesActor->SetVisibility(false);

  d->renderer->AddActor(d->cubeAxesActor.GetPointer());

  // Setup DepthMap actor
  d->depthScalarFilter->SetScalarArrayName(DepthMapArrays::TrueColor);
  vtkNew<vtkPolyDataMapper> mapper;
  mapper->SetInputConnection(d->depthScalarFilter->GetOutputPort());
  mapper->SetColorModeToDirectScalars();
  d->depthMapActor->SetMapper(mapper.GetPointer());
  d->renderer->AddActor(d->depthMapActor.GetPointer());
  d->depthMapActor->VisibilityOff();

  // Add keyboard actions for increasing and descreasing depth point size
  QAction* actionIncreasePointSize = new QAction(this);
  actionIncreasePointSize->setShortcut(Qt::Key_Plus);
  actionIncreasePointSize->setShortcutContext(Qt::WidgetWithChildrenShortcut);
  d->UI.renderWidget->addAction(actionIncreasePointSize);
  connect(actionIncreasePointSize, SIGNAL(triggered()),
    this, SLOT(increaseDepthMapPointSize()));

  QAction* actionDecreasePointSize = new QAction(this);
  actionDecreasePointSize->setShortcut(Qt::Key_Minus);
  actionDecreasePointSize->setShortcutContext(Qt::WidgetWithChildrenShortcut);
  d->UI.renderWidget->addAction(actionDecreasePointSize);
  connect(actionDecreasePointSize, SIGNAL(triggered()),
    this, SLOT(decreaseDepthMapPointSize()));
}

//-----------------------------------------------------------------------------
WorldView::~WorldView()
{
}

//-----------------------------------------------------------------------------
void WorldView::setBackgroundColor(QColor const& color)
{
  QTE_D();
  d->renderer->SetBackground(color.redF(), color.greenF(), color.blueF());
  d->UI.renderWidget->update();
}

//-----------------------------------------------------------------------------
void WorldView::setValidDepthInput(bool state)
{
  QTE_D();
  d->validDepthInput = state;
  if (!state)
  {
    d->depthMapActor->VisibilityOff();
  }
}

//-----------------------------------------------------------------------------
void WorldView::connectDepthPipeline()
{
  QTE_D();

  if (!d->inputDepthGeometryFilter)
  {
    d->depthScalarFilter->SetInputConnection(0);
    return;
  }

  switch (d->depthMapOptions->displayMode())
  {
  case DepthMapOptions::Points:
    d->inputDepthGeometryFilter->GenerateTriangleOutputOff();
    d->depthScalarFilter->SetInputConnection(
      d->inputDepthGeometryFilter->GetOutputPort(1));
    break;
  case DepthMapOptions::Surfaces:
    d->inputDepthGeometryFilter->GenerateTriangleOutputOn();
    d->depthScalarFilter->SetInputConnection(
      d->inputDepthGeometryFilter->GetOutputPort(2));
    break;
  }
}

//-----------------------------------------------------------------------------
void WorldView::setDepthGeometryFilter(vtkMaptkImageDataGeometryFilter* geometryFilter)
{
  QTE_D();

  if (d->inputDepthGeometryFilter != geometryFilter)
  {
    d->inputDepthGeometryFilter = geometryFilter;
    this->connectDepthPipeline();
  }
}

//-----------------------------------------------------------------------------
void WorldView::updateDepthMap()
{
  QTE_D();

  if (!d->depthMapActor->GetVisibility())
  {
    if (d->UI.actionShowDepthMap->isChecked())
    {
      d->depthMapActor->SetVisibility(true);
    }
    else
    {
      d->rangeUpdateNeeded = true;
      return;
    }
  }

  // d->rangeUpdateNeeded set to false in updateThresholdRanges
  this->updateThresholdRanges();

  d->UI.renderWidget->update();
}

//-----------------------------------------------------------------------------
void WorldView::updateThresholdRanges()
{
  QTE_D();

  if (!d->inputDepthGeometryFilter)
  {
    qWarning() << "Geometry filter is not set!";
    return;
  }

  d->rangeUpdateNeeded = false;

  // Update threhsold settings per user settings
  vtkAlgorithm* geometryInputFilter =
    d->inputDepthGeometryFilter->GetInputAlgorithm();
  geometryInputFilter->Update();

  if (!(d->depthMapOptions->isFilterEnabled() &&
        d->depthMapOptions->isFilterPersistent()))
  {
    // Initialize the filters
    double bcRange[2], urRange[2];

    vtkImageData* imageData = vtkImageData::SafeDownCast(
      geometryInputFilter->GetOutputDataObject(0));
    if (!imageData)
    {
      qWarning() << "Unexpected input type to geometry filter!";
    }

    auto const pd = imageData->GetPointData();
    auto const bcArray = pd->GetArray(DepthMapArrays::BestCostValues);
    auto const urArray = pd->GetArray(DepthMapArrays::UniquenessRatios);

    if (bcArray && urArray)
    {
      bcArray->GetRange(bcRange);
      urArray->GetRange(urRange);

      d->depthMapOptions->initializeFilters(bcRange[0], bcRange[1],
                                            urRange[0], urRange[1]);
    }
    else
    {
      qWarning() << "Failed to load data from depth map";
    }
  }

}

//-----------------------------------------------------------------------------
void WorldView::initFrameSampling(int nbFrames)
{
  QTE_D();

  d->volumeOptions->initFrameSampling(nbFrames);
}

//-----------------------------------------------------------------------------
void WorldView::loadVolume(QString path, QString krtd, QString frame)
{
  QTE_D();

  d->UI.actionShowVolume->setEnabled(true);

  std::string filename = path.toStdString();

  // Create the vtk pipeline
  // Read volume
  vtkNew<vtkXMLStructuredGridReader> readerV;
  readerV->SetFileName(filename.c_str());

  d->volume = readerV->GetOutput();
  // Transform cell data to point data for contour filter
  vtkNew<vtkCellDataToPointData> transformCellToPointData;
  transformCellToPointData->SetInputConnection(readerV->GetOutputPort());
  transformCellToPointData->PassCellDataOn();

  // Apply contour
  d->contourFilter = vtkContourFilter::New();
  d->contourFilter->SetInputConnection(transformCellToPointData->GetOutputPort());
  d->contourFilter->SetNumberOfContours(1);
  d->contourFilter->SetValue(0, 0.5);
  // Declare which table will be use for the contour
  d->contourFilter->SetInputArrayToProcess(0, 0, 0,
                                           vtkDataObject::FIELD_ASSOCIATION_POINTS,
                                           "reconstruction_scalar");

  // Create mapper
  vtkNew<vtkPolyDataMapper> contourMapper;
  contourMapper->SetInputConnection(d->contourFilter->GetOutputPort());
  contourMapper->SetColorModeToDirectScalars();

  // Set the actor's mapper
  d->volumeActor->SetMapper(contourMapper.Get());
  d->volumeActor->SetVisibility(false);
  d->volumeOptions->setActor(d->volumeActor.Get());
  d->volumeOptions->setKrtdFrameFile(krtd, frame);


  // Add this actor to the renderer
  d->renderer->AddActor(d->volumeActor.Get());
  emit(contourChanged());
}

//-----------------------------------------------------------------------------
void WorldView::setVolumeVisible(bool state)
{
  QTE_D();

  d->volumeActor->SetVisibility(state);
  d->volumeOptions->setEnabled(state);
  d->UI.renderWidget->update();
}

//-----------------------------------------------------------------------------
void WorldView::setVolumeCurrentFramePath(QString path)
{
  QTE_D();

  d->volumeOptions->setCurrentFramePath(path.toStdString());
}

//-----------------------------------------------------------------------------
void WorldView::computeContour(double threshold)
{
  QTE_D();

  d->contourFilter->SetValue(0, threshold);
  d->UI.renderWidget->update();

  if(d->volumeOptions->isColorOptionsEnabled())
  {
    d->volumeOptions->colorize();
  }

}

//-----------------------------------------------------------------------------
void WorldView::addCamera(int id, vtkMaptkCamera* camera)
{
  Q_UNUSED(id)

  QTE_D();

  d->cameraRep->AddCamera(id, camera);

  d->updateCameras(this);
  d->updateAxes(this);
}

//-----------------------------------------------------------------------------
void WorldView::removeCamera(int id)
{
  Q_UNUSED(id)

    QTE_D();

  d->cameraRep->RemoveCamera(id);
}

//-----------------------------------------------------------------------------
void WorldView::setActiveCamera(int id)
{
  static auto const plane = kwiver::vital::vector_4d(0.0, 0.0, 1.0, 0.0);

  QTE_D();

  d->cameraRep->SetActiveCamera(id);
  vtkMaptkCamera * camera = dynamic_cast<vtkMaptkCamera*>(d->cameraRep->GetActiveCamera());

  if (camera)
  {
    camera->GetTransform(d->imageProjection.GetPointer(), plane.data());
    d->updateImageTransform();

    auto const showImage = d->UI.actionShowFrameImage->isChecked();
    d->validTransform = true;
    d->imageActor->SetVisibility(d->validImage && showImage);
  }
  else
  {
    d->validTransform = false;
    d->imageActor->SetVisibility(false);
  }

  d->updateCameras(this);
  d->updateAxes(this);
}

//-----------------------------------------------------------------------------
void WorldView::setImageData(vtkImageData* data, QSize const& dimensions)
{
  QTE_D();

  d->imageLocalTransform->Identity();
  d->imageLocalTransform->SetElement(1, 1, -1.0);
  d->imageLocalTransform->SetElement(1, 3, dimensions.height());
  d->updateImageTransform();

  auto const showImage = d->UI.actionShowFrameImage->isChecked();
  d->validImage = data;
  d->imageActor->SetInputData(data ? data : d->emptyImage.GetPointer());
  d->imageActor->SetVisibility(data && d->validTransform && showImage);
  d->UI.renderWidget->update();
}

//-----------------------------------------------------------------------------
void WorldView::setLandmarks(kwiver::vital::landmark_map const& lm)
{
  QTE_D();

  auto const& landmarks = lm.landmarks();
  auto const size = static_cast<vtkIdType>(landmarks.size());

  auto const defaultColor = kwiver::vital::rgb_color{};
  auto haveColor = false;
  auto maxObservations = unsigned{0};
  auto minZ = qInf(), maxZ = -qInf();

  d->landmarkPoints->Reset();
  d->landmarkVerts->Reset();
  d->landmarkColors->Reset();
  d->landmarkElevations->Reset();
  d->landmarkObservations->Reset();
  d->landmarkPoints->Allocate(size);
  d->landmarkVerts->Allocate(size);
  d->landmarkColors->Allocate(3 * size);
  d->landmarkElevations->Allocate(size);
  d->landmarkObservations->Allocate(size);

  vtkIdType vertIndex = 0;
  foreach (auto const& lm, landmarks)
  {
    auto const& pos = lm.second->loc();
    auto const& color = lm.second->color();
    auto const observations = lm.second->observations();

    d->landmarkPoints->InsertNextPoint(pos.data());
    d->landmarkVerts->InsertNextCell(1);
    d->landmarkVerts->InsertCellPoint(vertIndex++);
    d->landmarkColors->InsertNextValue(color.r);
    d->landmarkColors->InsertNextValue(color.g);
    d->landmarkColors->InsertNextValue(color.b);
    d->landmarkElevations->InsertNextValue(pos[2]);
    d->landmarkObservations->InsertNextValue(observations);

    haveColor = haveColor || (color != defaultColor);
    maxObservations = qMax(maxObservations, observations);
    minZ = qMin(minZ, pos[2]);
    maxZ = qMax(maxZ, pos[2]);
  }

  auto fields = QHash<QString, FieldInformation>{};
  fields.insert("Elevation", FieldInformation{Elevation, {minZ, maxZ}});
  if (maxObservations)
  {
    auto const upper = static_cast<double>(maxObservations);
    fields.insert("Observations", FieldInformation{Observations, {0.0, upper}});
  }

  d->landmarkOptions->setTrueColorAvailable(haveColor);
  d->landmarkOptions->setDataFields(fields);

  d->landmarkPoints->Modified();
  d->landmarkVerts->Modified();
  d->landmarkColors->Modified();
  d->landmarkObservations->Modified();

  d->updateScale(this);
  d->updateAxes(this);
}

//-----------------------------------------------------------------------------
void WorldView::setImageVisible(bool state)
{
  QTE_D();

  d->imageActor->SetVisibility(state && d->validImage && d->validTransform);
  d->updateAxes(this, true);
}

//-----------------------------------------------------------------------------
void WorldView::setCamerasVisible(bool state)
{
  QTE_D();
  d->cameraOptions->setCamerasVisible(state);
  d->updateAxes(this, true);
}

//-----------------------------------------------------------------------------
void WorldView::setLandmarksVisible(bool state)
{
  QTE_D();

  d->landmarkActor->SetVisibility(state);
  d->updateAxes(this, true);
}

//-----------------------------------------------------------------------------
void WorldView::setGroundPlaneVisible(bool state)
{
  QTE_D();

  d->groundActor->SetVisibility(state);
  d->updateAxes(this, true);
}

//-----------------------------------------------------------------------------
void WorldView::setAxesVisible(bool state)
{
  QTE_D();

  d->axesVisible = state;

  if (state)
  {
    d->updateAxes(this, true);
  }
  else
  {
    d->cubeAxesActor->SetVisibility(state);
    d->UI.renderWidget->update();
  }
}

//-----------------------------------------------------------------------------
void WorldView::setDepthMapVisible(bool state)
{
  QTE_D();

  if (d->validDepthInput &&
      d->depthMapActor->GetVisibility() != static_cast<int>(state))
  {
    d->depthMapActor->SetVisibility(state);

    if (d->rangeUpdateNeeded && state)
    {
      this->updateThresholdRanges();
    }
    d->UI.renderWidget->update();
  }
  d->depthMapOptions->setEnabled(state);
}

//-----------------------------------------------------------------------------
void WorldView::queueResetView()
{
  QMetaObject::invokeMethod(this, "resetView", Qt::QueuedConnection);
}

//-----------------------------------------------------------------------------
void WorldView::resetView()
{
  QTE_D();

  d->renderer->ResetCamera();
  d->UI.renderWidget->update();
}

//-----------------------------------------------------------------------------
void WorldView::resetViewToLandmarks()
{
  QTE_D();

  vtkBoundingBox bbox;
  bbox.AddBounds(d->landmarkActor->GetBounds());

  double bounds[6];
  bbox.GetBounds(bounds);
  d->renderer->ResetCamera(bounds);
  d->renderer->ResetCameraClippingRange();

  d->UI.renderWidget->update();
}

//-----------------------------------------------------------------------------
void WorldView::viewToWorldTop()
{
  QTE_D();
  d->setView(0.0, 0.0, +1.0, 0.0, +1.0, 0.0);
}

//-----------------------------------------------------------------------------
void WorldView::viewToWorldLeft()
{
  QTE_D();
  d->setView(-1.0, 0.0, 0.0, 0.0, 0.0, +1.0);
}

//-----------------------------------------------------------------------------
void WorldView::viewToWorldRight()
{
  QTE_D();
  d->setView(+1.0, 0.0, 0.0, 0.0, 0.0, +1.0);
}

//-----------------------------------------------------------------------------
void WorldView::viewToWorldFront()
{
  QTE_D();
  d->setView(0.0, -1.0, 0.0, 0.0, 0.0, +1.0);
}

//-----------------------------------------------------------------------------
void WorldView::viewToWorldBack()
{
  QTE_D();
  d->setView(0.0, +1.0, 0.0, 0.0, 0.0, +1.0);
}

//-----------------------------------------------------------------------------
void WorldView::setPerspective(bool perspective)
{
  QTE_D();

  d->renderer->GetActiveCamera()->SetParallelProjection(!perspective);
  d->UI.renderWidget->update();
}

//-----------------------------------------------------------------------------
void WorldView::invalidateGeometry()
{
  QTE_D();
  d->updateAxes(this);
}

//-----------------------------------------------------------------------------
void WorldView::updateCameras()
{
  QTE_D();

  if (d->cameraRepDirty)
  {
    d->cameraRep->Update();
    d->UI.renderWidget->update();
    d->cameraRepDirty = false;
  }
}

//-----------------------------------------------------------------------------
void WorldView::updateAxes()
{
  QTE_D();

  if (d->axesDirty)
  {
    // Compute bounds of visible actors
    auto const props = d->renderer->GetViewProps();
    vtkBoundingBox bbox;

    double tb[6];
    d->imageActor->GetBounds(tb);

    props->InitTraversal();
    while (auto const prop = props->GetNextProp())
    {
      // Skip the axes, ground and frustums representations, and any hidden
      // actors
      if (prop == d->cubeAxesActor.GetPointer() ||
          prop == d->groundActor.GetPointer() ||
          prop == d->cameraRep->GetActiveActor() ||
          prop == d->cameraRep->GetNonActiveActor() ||
          !prop->GetVisibility())
      {
        continue;
      }

      bbox.AddBounds(prop->GetBounds());
    }

    // Update scale of axes, or hide if nothing is visible
    if (bbox.IsValid())
    {
      d->cubeAxesActor->SetVisibility(d->axesVisible);
      d->cubeAxesActor->SetCamera(d->renderer->GetActiveCamera());
      d->cubeAxesActor->SetBounds(bbox.GetBound(0), bbox.GetBound(1),
                                  bbox.GetBound(2), bbox.GetBound(3),
                                  bbox.GetBound(4), bbox.GetBound(5));
    }
    else
    {
      d->cubeAxesActor->SetVisibility(false);
    }

    d->axesDirty = false;
    d->renderer->ResetCameraClippingRange();
    d->UI.renderWidget->update();
  }
}

//-----------------------------------------------------------------------------
void WorldView::updateScale()
{
  QTE_D();

  if (d->scaleDirty)
  {
    // Make sure the cameras are updated so that we will get the correct bounds
    // for them
    this->updateCameras();

    // Determine a base scale factor for the camera frustums... for now, using
    // the diagonal of the extents of the landmarks and camera centers
    vtkBoundingBox bbox;

    // Add landmarks
    d->landmarkActor->GetMapper()->Update();
    bbox.AddBounds(d->landmarkActor->GetBounds());

    // If landmarks are not valid, then get ground scale from the cameras
    if (!bbox.IsValid())
    {
      // Add camera centers
      bbox.AddBounds(d->cameraRep->GetPathActor()->GetBounds());
    }

    // Update ground plane scale
    auto const groundScale =
      1.5 * qMax(qMax(qAbs(bbox.GetBound(0)), qAbs(bbox.GetBound(1))),
                 qMax(qAbs(bbox.GetBound(2)), qAbs(bbox.GetBound(3))));

    // Add camera centers
    bbox.AddBounds(d->cameraRep->GetPathActor()->GetBounds());

    // Check if we have geometry (we can still get here if a scale update was
    // triggered by setting the active camera with only images loaded)
    if (bbox.IsValid())
    {
      // Compute base scale (20% of scale factor)
      auto const cameraScale = 0.2 * bbox.GetDiagonalLength();

      // Update camera scale
      d->cameraOptions->setBaseCameraScale(cameraScale);

      // Update the ground scale
      d->groundPlane->SetOrigin(-groundScale, -groundScale, 0.0);
      d->groundPlane->SetPoint1(+groundScale, -groundScale, 0.0);
      d->groundPlane->SetPoint2(-groundScale, +groundScale, 0.0);
      d->groundPlane->Modified();
    }

    d->scaleDirty = false;

    // If we're updating the scale, it's probably because the geometry changed,
    // which means now (but AFTER updating the camera scales!) is a good time
    // to ensure that the clipping planes are set reasonably
    d->renderer->ResetCameraClippingRange();
  }
}

//-----------------------------------------------------------------------------
void WorldView::updateDepthMapDisplayMode()
{
  QTE_D();

  this->connectDepthPipeline();
  d->UI.renderWidget->update();
}

//-----------------------------------------------------------------------------
void WorldView::updateDepthMapThresholds(bool filterState)
{
  QTE_D();

  double bestCostValueMin = d->depthMapOptions->bestCostValueMinimum();
  double bestCostValueMax = d->depthMapOptions->bestCostValueMaximum();
  double uniquenessRatioMin = d->depthMapOptions->uniquenessRatioMinimum();
  double uniquenessRatioMax = d->depthMapOptions->uniquenessRatioMaximum();

  d->inputDepthGeometryFilter->SetConstraint(
    DepthMapArrays::BestCostValues, bestCostValueMin, bestCostValueMax);
  d->inputDepthGeometryFilter->SetConstraint(
    DepthMapArrays::UniquenessRatios, uniquenessRatioMin, uniquenessRatioMax);
  d->inputDepthGeometryFilter->SetThresholdCells(filterState);

  emit depthMapThresholdsChanged();

  d->UI.renderWidget->update();
}

//-----------------------------------------------------------------------------
void WorldView::saveDepthPoints(QString const& path)
{
  QTE_D();

  vtkNew<vtkPLYWriter> writer;

  writer->SetFileName(path.toStdString().c_str());
  writer->SetInputConnection(d->depthScalarFilter->GetOutputPort());
  writer->SetColorMode(0);
  writer->SetArrayName(DepthMapArrays::TrueColor);
  writer->Write();
}

//-----------------------------------------------------------------------------
void WorldView::exportWebGLScene(QString const& path)
{
#ifdef VTKWEBGLEXPORTER
  QTE_D();

  vtkNew<vtkWebGLExporter> exporter;

  int width = d->renderWindow->GetScreenSize()[0];
  int height = d->renderWindow->GetScreenSize()[1];

  // HACK: There's a bug in vtkWebGLPolyData->GetColorsFromPointData():
  // LookupTable->GetVectorMode() is passed instead of GetMapper->GetColorMode.
  // The workaround here is to force the LookupTable's VectorMode to
  // vtkScalarsToColors::RGBCOLORS if the mapper's ColorMode equals
  // VTK_COLOR_MODE_DEFAULT or VTK_COLOR_MODE_DIRECT_SCALARS.
  //
  // See also https://gitlab.kitware.com/vtk/vtk/merge_requests/1585. This
  // workaround should be removed when MapTK moves to a version of VTK that
  // contains the fix.
  auto const actors = d->renderer->GetActors();
  actors->InitTraversal();
  while (auto const actor = actors->GetNextActor())
  {
    if(actor && actor->GetMapper() &&
       (actor->GetMapper()->GetColorMode() == VTK_COLOR_MODE_DEFAULT ||
        actor->GetMapper()->GetColorMode() == VTK_COLOR_MODE_DIRECT_SCALARS))
    {
      actor->GetMapper()->GetLookupTable()->SetVectorModeToRGBColors();
    }
  }

  exporter->exportStaticScene(d->renderWindow->GetRenderers(), width, height,
                              qPrintable(path));
#else
  Q_UNUSED(path)
#endif
}

//-----------------------------------------------------------------------------
void WorldView::saveMesh(const QString &path)
{
  QTE_D();

  vtkPolyData* mesh = d->contourFilter->GetOutput();

  for (int i = 0; i < mesh->GetPointData()->GetNumberOfArrays(); ++i)
  {
    mesh->GetPointData()->RemoveArray(i);;
  }

  vtkNew<vtkXMLPolyDataWriter> writer;

  writer->SetFileName(path.toStdString().c_str());
  writer->AddInputDataObject(mesh);
  writer->SetDataModeToBinary();
  writer->Write();

  std::cout << "Saved : " << path.toStdString() << std::endl;

}

//-----------------------------------------------------------------------------
void WorldView::saveVolume(const QString &path)
{
  QTE_D();

  //NOTE: For now, the volume is set in the configuration parameters.
  //      It may be generated directly from the GUI in the future.

  vtkNew<vtkXMLStructuredGridWriter> writer;

  writer->SetFileName(path.toStdString().c_str());
  writer->AddInputDataObject(d->volume);
  writer->SetDataModeToBinary();
  writer->Write();

  std::cout << "Saved : " << path.toStdString() << std::endl;
}

//-----------------------------------------------------------------------------
void WorldView::saveColoredMesh(const QString &path)
{
  QTE_D();

  const QString ext = QFileInfo(path).suffix().toLower();
  if(ext == "ply")
  {
    vtkNew<vtkPLYWriter> writer;
    writer->SetFileName(path.toStdString().c_str());
    writer->SetColorMode(0);
    vtkSmartPointer<vtkPolyData> mesh = d->contourFilter->GetOutput();
    writer->SetArrayName(mesh->GetPointData()->GetScalars()->GetName());
    writer->SetLookupTable(d->volumeActor->GetMapper()->GetLookupTable());
    writer->AddInputDataObject(mesh);
    writer->Write();
  }
  else
  {
    vtkNew<vtkXMLPolyDataWriter> writer;
    writer->SetFileName(path.toStdString().c_str());
    writer->SetDataModeToBinary();
    writer->AddInputDataObject(d->contourFilter->GetOutput());
    writer->Write();
  }

  std::cout << "Saved : " << path.toStdString() << std::endl;
}

//-----------------------------------------------------------------------------
void WorldView::increaseDepthMapPointSize()
{
  QTE_D();

  float pointSize = d->depthMapActor->GetProperty()->GetPointSize();
  d->depthMapActor->GetProperty()->SetPointSize(pointSize + 0.5);

  d->UI.renderWidget->update();
}

//-----------------------------------------------------------------------------
void WorldView::decreaseDepthMapPointSize()
{
  QTE_D();

  float pointSize = d->depthMapActor->GetProperty()->GetPointSize() - 0.5;
  d->depthMapActor->GetProperty()->SetPointSize(pointSize < 1 ? 1 : pointSize);

  d->UI.renderWidget->update();
}
