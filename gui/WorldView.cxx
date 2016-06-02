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

#include "WorldView.h"

#include "ui_WorldView.h"
#include "am_WorldView.h"

#include "CameraOptions.h"
#include "FieldInformation.h"
#include "ImageOptions.h"
#include "PointOptions.h"
#include "DepthMapOptions.h"
#include "VolumeOptions.h"
#include "vtkMaptkCamera.h"
#include "vtkMaptkCameraRepresentation.h"

#include <vital/types/camera.h>
#include <vital/types/landmark_map.h>

#include <vtkActorCollection.h>
#include <vtkBoundingBox.h>
#include <vtkCellArray.h>
#include <vtkCellDataToPointData.h>
#include <vtkContourFilter.h>
#include <vtkDoubleArray.h>
#include <vtkImageActor.h>
#include <vtkImageData.h>
#include <vtkMatrix4x4.h>
#include <vtkNew.h>
#include <vtkPlaneSource.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkUnsignedCharArray.h>
#include <vtkUnsignedIntArray.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkStructuredGrid.h>
#include <vtkXMLStructuredGridReader.h>
#include <vtkGeometryFilter.h>
#include <vtkThreshold.h>
#include <vtkDataSetTriangleFilter.h>
#include <vtkProjectedTetrahedraMapper.h>
#include <vtkPiecewiseFunction.h>
#include <vtkColorTransferFunction.h>
#include <vtkUnstructuredGrid.h>
#include <vtkVolumeProperty.h>
#include <vtkXMLImageDataReader.h>
#include <vtkThreshold.h>
#include <vtkThresholdPoints.h>
#include <vtkInformation.h>
#include <vtkCubeAxesActor.h>
#include <vtkTextProperty.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkXMLStructuredGridWriter.h>

#include <qtMath.h>

#include <QtGui/QMenu>
#include <QtGui/QToolButton>
#include <QtGui/QWidgetAction>

namespace // anonymous
{
static char const* const TrueColor = "truecolor";
static char const* const Elevation = "elevation";
static char const* const Observations = "observations";
}

//-----------------------------------------------------------------------------
class WorldViewPrivate
{
public:
  WorldViewPrivate() :
    validImage(false),
    validTransform(false),
    cameraRepDirty(false),
    scaleDirty(false)
    {}

  void setPopup(QAction* action, QMenu* menu);
  void setPopup(QAction* action, QWidget* widget);

  void setView(kwiver::vital::vector_3d const& normal,
               kwiver::vital::vector_3d const& up);
  void setView(double ni, double nj, double nk,
               double ui, double uj, double uk);

  void updateImageTransform();
  void updateCameras(WorldView*);
  void updateScale(WorldView*);

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

  ImageOptions* imageOptions;
  CameraOptions* cameraOptions;
  PointOptions* landmarkOptions;
  DepthMapOptions* depthMapOptions;
  VolumeOptions* volumeOptions;
  vtkContourFilter* contourFilter;

  vtkNew<vtkMatrix4x4> imageProjection;
  vtkNew<vtkMatrix4x4> imageLocalTransform;

  vtkSmartPointer<vtkPolyData> currentDepthmap;

  vtkNew<vtkActor> polyDataActor;
  vtkNew<vtkActor> depthmapActor;
  vtkNew<vtkActor> volumeActor;
  vtkStructuredGrid* volume;
  vtkNew<vtkCubeAxesActor> cubeAxesActor;

  std::map<int, std::string> dMList;
  std::map<int, std::string> dMListSG;
  std::map<int, std::string> dMListVert;

bool validImage;
  bool validTransform;

  bool cameraRepDirty;
  bool scaleDirty;
};

QTE_IMPLEMENT_D_FUNC(WorldView)

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
          d->UI.renderWidget, SLOT(update()));

  d->landmarkOptions = new PointOptions("WorldView/Landmarks", this);
  d->landmarkOptions->addActor(d->landmarkActor.GetPointer());
  d->setPopup(d->UI.actionShowLandmarks, d->landmarkOptions);

  connect(d->landmarkOptions, SIGNAL(modified()),
          d->UI.renderWidget, SLOT(update()));

  d->depthMapOptions = new DepthMapOptions("WorldView/Depthmap", this);
  d->setPopup(d->UI.actionDepthmapDisplay, d->depthMapOptions);

  d->depthMapOptions->addActor("points",d->polyDataActor.Get());
  d->depthMapOptions->addActor("surfaces",d->depthmapActor.Get());

  d->volumeOptions = new VolumeOptions("WorldView/Volume", this);
  d->setPopup(d->UI.actionVolumeDisplay, d->volumeOptions);


  connect(d->volumeOptions, SIGNAL(modified()),
          d->UI.renderWidget, SLOT(update()));

  connect(d->depthMapOptions, SIGNAL(depthMapRepresentationChanged()),
          this, SLOT(updateDepthMapRepresentation()));

  connect(d->depthMapOptions, SIGNAL(depthMapThresholdsChanged()),
          this, SLOT(updateDepthMapThresholds()));

  connect(d->depthMapOptions, SIGNAL(bBoxToggled()),
          this, SLOT(setGridVisible()));

  d->depthMapOptions->setEnabled(false);

  connect(this, SIGNAL(contourChanged()),
          d->UI.renderWidget, SLOT(update()));

  connect(d->UI.actionVolumeDisplay, SIGNAL(triggered(bool)),
          this, SIGNAL(meshEnabled(bool)));
  connect(d->volumeOptions, SIGNAL(colorOptionsEnabled(bool)),
          this, SIGNAL(coloredMeshEnabled(bool)));

  // Connect actions
  this->addAction(d->UI.actionViewReset);
  this->addAction(d->UI.actionViewResetLandmarks);
  this->addAction(d->UI.actionViewPerspective);
  this->addAction(d->UI.actionShowCameras);
  this->addAction(d->UI.actionShowLandmarks);
  this->addAction(d->UI.actionShowGroundPlane);
  this->addAction(d->UI.actionDepthmapDisplay);
  this->addAction(d->UI.actionVolumeDisplay);

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
  connect(d->UI.actionDepthmapDisplay, SIGNAL(toggled(bool)),
          this, SLOT(setDepthMapVisible(bool)));
  connect(d->UI.actionVolumeDisplay, SIGNAL(toggled(bool)),
          this, SLOT(setVolumeVisible(bool)));

  connect(d->volumeOptions, SIGNAL(meshIsColorizedFromColorizeSurfaceOption),
    d->UI.renderWidget, SLOT(update()));

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
std::string WorldView::getActiveDepthMapType()
{
  QTE_D();

  if (d->depthMapOptions->isPointsChecked()) {
    return "points";
  }
  if (d->depthMapOptions->isSurfacesChecked()) {
    return "surfaces";
  }

  return "";
}

//----------------------------------------------------------------------

void WorldView::loadVolume(QString path, int nbFrames, QString krtd, QString vti)
{
  QTE_D();

  d->volumeOptions->initFrameSampling(nbFrames);

  d->UI.actionVolumeDisplay->setEnabled(true);

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
  d->contourFilter->SetInputArrayToProcess(0, 0, 0, vtkDataObject::FIELD_ASSOCIATION_POINTS, "reconstruction_scalar");

  // Create mapper
  vtkNew<vtkPolyDataMapper> contourMapper;
  contourMapper->SetInputConnection(d->contourFilter->GetOutputPort());
  contourMapper->SetColorModeToDirectScalars();

  // Set the actor's mapper
  d->volumeActor->SetMapper(contourMapper.Get());
  d->volumeActor->SetVisibility(false);
  d->volumeOptions->setActor(d->volumeActor.Get());
  d->volumeOptions->setKrtdVtiFile(krtd, vti);


  // Add this actor to the renderer
  d->renderer->AddActor(d->volumeActor.Get());
  emit(contourChanged());
}

//-----------------------------------------------------------------------------
void WorldView::setActiveDepthMap(vtkMaptkCamera* camera, QString vtiPath) {
  QTE_D();

  currentVtiPath = vtiPath;

  if (d->UI.actionDepthmapDisplay->isChecked() && !currentVtiPath.isEmpty()) {

    std::string filename = currentVtiPath.toStdString();

    vtkNew<vtkXMLImageDataReader> readerIm;

    readerIm->SetFileName(filename.c_str());
    readerIm->Update();

    int width = readerIm->GetOutput()->GetDimensions()[0];
    int height = readerIm->GetOutput()->GetDimensions()[1];

    vtkNew<vtkPoints> points;
    points->SetNumberOfPoints(width*height);

    double dmToImageRatio = camera->GetImageDimensions()[0]/width;

    double pixel[3];
    for (vtkIdType idPixel = 0; idPixel < readerIm->GetOutput()->GetNumberOfPoints(); ++idPixel) {
      readerIm->GetOutput()->GetPoint(idPixel,pixel);
      pixel[1] = height-1-pixel[1];

      kwiver::vital::vector_3d p;
      double depth = readerIm->GetOutput()->GetPointData()->GetArray("Depths")->GetTuple1(idPixel);

      pixel[0] *= dmToImageRatio;
      pixel[1] *= dmToImageRatio;
      camera->UnprojectPoint(pixel, depth, &p);


      points->SetPoint(idPixel,p[0],p[1],p[2]);

    }

    vtkSmartPointer<vtkPolyData> polyData;

    vtkNew<vtkGeometryFilter> geometryFilterIm;
    geometryFilterIm->SetInputData(readerIm->GetOutput());
    geometryFilterIm->Update();
    polyData = geometryFilterIm->GetOutput();

    polyData->SetPoints(points.Get());


    polyData->Print(std::cout);


    d->currentDepthmap = polyData.Get();
    d->currentDepthmap->GetPointData()->SetScalars(d->currentDepthmap->GetPointData()->GetArray("Color"));
    vtkNew<vtkPolyDataMapper> mapper;

    mapper->SetInputData(d->currentDepthmap.Get());
    mapper->SetColorModeToDirectScalars();

    d->depthmapActor->SetMapper(mapper.Get());
    d->depthmapActor->SetVisibility(true);

    if(d->depthMapOptions->isPointsChecked()) {
      d->depthmapActor->GetProperty()->SetRepresentationToPoints();
    }
    else {
      d->depthmapActor->GetProperty()->SetRepresentationToSurface();
    }

    d->renderer->AddActor(d->depthmapActor.Get());

    d->renderer->AddViewProp(d->depthmapActor.GetPointer());

    //Calculating the bounding box for the grid coordinates
    d->cubeAxesActor->SetBounds(d->currentDepthmap->GetBounds());
    d->cubeAxesActor->SetCamera(d->renderer->GetActiveCamera());
    d->cubeAxesActor->GetTitleTextProperty(0)->SetColor(1.0, 0.0, 0.0);
    d->cubeAxesActor->GetLabelTextProperty(0)->SetColor(1.0, 0.0, 0.0);

    d->cubeAxesActor->GetTitleTextProperty(1)->SetColor(0.0, 1.0, 0.0);
    d->cubeAxesActor->GetLabelTextProperty(1)->SetColor(0.0, 1.0, 0.0);

    d->cubeAxesActor->GetTitleTextProperty(2)->SetColor(0.0, 0.0, 1.0);
    d->cubeAxesActor->GetLabelTextProperty(2)->SetColor(0.0, 0.0, 1.0);

    d->cubeAxesActor->DrawXGridlinesOn();
    d->cubeAxesActor->DrawYGridlinesOn();
    d->cubeAxesActor->DrawZGridlinesOn();
    d->cubeAxesActor->SetGridLineLocation(VTK_GRID_LINES_FURTHEST);

    d->cubeAxesActor->XAxisMinorTickVisibilityOff();
    d->cubeAxesActor->YAxisMinorTickVisibilityOff();
    d->cubeAxesActor->ZAxisMinorTickVisibilityOff();

    d->renderer->AddActor(d->cubeAxesActor.Get());
    d->cubeAxesActor->SetVisibility(d->depthMapOptions->isBBoxChecked());
    d->renderer->GetActiveCamera()->Azimuth(30);
    d->renderer->GetActiveCamera()->Elevation(30);

    //initializing the filters
    double bcRange[2], urRange[2];
    polyData->GetPointData()->GetArray("Best Cost Values")->GetRange(bcRange);
    polyData->GetPointData()->GetArray("Uniqueness Ratios")->GetRange(urRange);

    d->depthMapOptions->initializeFilters(bcRange[0],bcRange[1],urRange[0],urRange[1]);
  }

  currentCam = camera;


}

//-----------------------------------------------------------------------------
void WorldView::enableDepthMap()
{
  QTE_D();

  d->UI.actionDepthmapDisplay->setEnabled(true);
  d->UI.actionDepthmapDisplay->setCheckable(true);
  d->UI.actionDepthmapDisplay->setChecked(false);

  d->depthMapOptions->enable();
}

//-----------------------------------------------------------------------------

void WorldView::addCamera(int id, vtkMaptkCamera* camera)
{
  Q_UNUSED(id)

  QTE_D();

  d->cameraRep->AddCamera(camera);

  d->updateCameras(this);
}

//-----------------------------------------------------------------------------
void WorldView::setActiveCamera(vtkMaptkCamera* camera)
{
  static auto const plane = kwiver::vital::vector_4d(0.0, 0.0, 1.0, 0.0);

  QTE_D();

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

  d->cameraRep->SetActiveCamera(camera);

  d->updateCameras(this);
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
  fields.insert("Elevation", {Elevation, {minZ, maxZ}});
  if (maxObservations)
  {
    auto const upper = static_cast<double>(maxObservations);
    fields.insert("Observations", {Observations, {0.0, upper}});
  }

  d->landmarkOptions->setTrueColorAvailable(haveColor);
  d->landmarkOptions->setDataFields(fields);

  d->landmarkPoints->Modified();
  d->landmarkVerts->Modified();
  d->landmarkColors->Modified();
  d->landmarkObservations->Modified();

  d->updateScale(this);
}

//-----------------------------------------------------------------------------
void WorldView::setImageVisible(bool state)
{
  QTE_D();

  d->imageActor->SetVisibility(state && d->validImage && d->validTransform);
  d->UI.renderWidget->update();
}

//-----------------------------------------------------------------------------
void WorldView::setCamerasVisible(bool state)
{
  QTE_D();
  d->cameraOptions->setCamerasVisible(state);
}

//-----------------------------------------------------------------------------
void WorldView::setLandmarksVisible(bool state)
{
  QTE_D();

  d->landmarkActor->SetVisibility(state);
  d->UI.renderWidget->update();
}

//-----------------------------------------------------------------------------
void WorldView::setGroundPlaneVisible(bool state)
{
  QTE_D();

  d->groundActor->SetVisibility(state);
  d->UI.renderWidget->update();
}

//-----------------------------------------------------------------------------
void WorldView::setDepthMapVisible(bool state)
{
  QTE_D();

    d->depthmapActor->SetVisibility(state);
    d->depthMapOptions->setEnabled(state);

    if (state)
    {
      setGridVisible();
    }
    else
    {
      d->cubeAxesActor->SetVisibility(false);
    }

  setActiveDepthMap(currentCam,currentVtiPath);
  d->UI.renderWidget->update();
}

//-----------------------------------------------------------------------------
void WorldView::setGridVisible()
{
   QTE_D();

   if (d->UI.actionDepthmapDisplay->isChecked()) {
     d->cubeAxesActor->SetVisibility(d->depthMapOptions->isBBoxChecked());
   }
}
void WorldView::setVolumeVisible(bool state)
{
  QTE_D();
  d->volumeActor->SetVisibility(state);
  d->UI.renderWidget->update();
}

void WorldView::setVolumeCurrentDepthmapPath(QString path)
{
  QTE_D();

  d->volumeOptions->setCurrentDepthmapPath(path.toStdString());
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

void WorldView::saveColoredMesh(const QString &path)
{
  QTE_D();

  vtkPolyData* mesh = d->contourFilter->GetOutput();

  vtkNew<vtkXMLPolyDataWriter> writer;

  writer->SetFileName(path.toStdString().c_str());
  writer->AddInputDataObject(mesh);
  writer->SetDataModeToBinary();
  writer->Write();

  std::cout << "Saved : " << path.toStdString() << std::endl;
}

//-----------------------------------------------------------------------------
void WorldView::setPerspective(bool perspective)
{
  QTE_D();

  d->renderer->GetActiveCamera()->SetParallelProjection(!perspective);
  d->UI.renderWidget->update();
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

    // Update ground plane scale
    auto const groundScale =
      1.5 * qMax(qMax(qAbs(bbox.GetBound(0)), qAbs(bbox.GetBound(1))),
                 qMax(qAbs(bbox.GetBound(2)), qAbs(bbox.GetBound(3))));

    d->groundPlane->SetOrigin(-groundScale, -groundScale, 0.0);
    d->groundPlane->SetPoint1(+groundScale, -groundScale, 0.0);
    d->groundPlane->SetPoint2(-groundScale, +groundScale, 0.0);
    d->groundPlane->Modified();

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
    }

    d->scaleDirty = false;

    // If we're updating the scale, it's probably because the geometry changed,
    // which means now (but AFTER updating the camera scales!) is a good time
    // to ensure that the clipping planes are set reasonably
    d->renderer->ResetCameraClippingRange();
  }
}

//-----------------------------------------------------------------------------
void WorldView::updateDepthMapRepresentation()
{
  QTE_D();
//  std::cout << "update depth map..." << *matrix <<std::endl;
//  setActiveDepthMap(currentCam,currentVtiPath);
  if(d->depthMapOptions->isPointsChecked()) {
    d->depthmapActor->GetProperty()->SetRepresentationToPoints();
  }
  else {
    d->depthmapActor->GetProperty()->SetRepresentationToSurface();
  }
  d->UI.renderWidget->update();
}

//-----------------------------------------------------------------------------
void WorldView::updateDepthMapThresholds()
{

  QTE_D();

  double bestCostValueMin = d->depthMapOptions->getBestCostValueMin();
  double bestCostValueMax = d->depthMapOptions->getBestCostValueMax();
  double uniquenessRatioMin = d->depthMapOptions->getUniquenessRatioMin();
  double uniquenessRatioMax = d->depthMapOptions->getUniquenessRatioMax();

  vtkNew<vtkThreshold> thresholdBestCostValues, thresholdUniquenessRatios;

  thresholdBestCostValues->SetInputData(d->currentDepthmap);

  thresholdBestCostValues->SetInputArrayToProcess(0,0,0,vtkDataObject::FIELD_ASSOCIATION_POINTS,"Best Cost Values");
  thresholdBestCostValues->ThresholdBetween(bestCostValueMin,bestCostValueMax);

  thresholdUniquenessRatios->SetInputConnection(thresholdBestCostValues->GetOutputPort());
  thresholdUniquenessRatios->SetInputArrayToProcess(0,0,0,vtkDataObject::FIELD_ASSOCIATION_POINTS,"Uniqueness Ratios");
  thresholdUniquenessRatios->ThresholdBetween(uniquenessRatioMin,uniquenessRatioMax);

  vtkNew<vtkGeometryFilter> geometryFilter;
  geometryFilter->SetInputConnection(thresholdUniquenessRatios->GetOutputPort());
//  geometryFilter->Update();
  d->depthmapActor->GetMapper()->SetInputConnection(geometryFilter->GetOutputPort());
  d->depthmapActor->GetMapper()->Update();
}

//-----------------------------------------------------------------------------
void WorldView::computeContour(double threshold)
{
  QTE_D();

  d->contourFilter->SetValue(0, threshold);
  d->UI.renderWidget->update();
  emit(contourChanged());
}
