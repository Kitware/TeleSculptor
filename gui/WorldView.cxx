// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#include "WorldView.h"

#include "ui_WorldView.h"
#include "am_WorldView.h"

#include "CameraOptions.h"
#include "DataArrays.h"
#include "DepthMapOptions.h"
#include "FieldInformation.h"
#include "GroundControlPointsWidget.h"
#include "ImageOptions.h"
#include "tools/MeshColoration.h"
#include "PointOptions.h"
#include "RulerOptions.h"
#include "RulerWidget.h"
#include "VolumeOptions.h"
#include "vtkMaptkCameraRepresentation.h"
#include "vtkMaptkImageUnprojectDepth.h"
#include "vtkMaptkInteractorStyle.h"
#include "vtkMaptkScalarDataFilter.h"

#include "arrows/vtk/vtkKwiverCamera.h"
#include <vital/algo/algorithm.txx>
#include <vital/algo/pointcloud_io.h>
#include <vital/types/camera.h>
#include <vital/types/landmark_map.h>

#include <vtkBoundingBox.h>
#include <vtkBox.h>
#include <vtkBoxRepresentation.h>
#include <vtkBoxWidget2.h>
#include <vtkCellArray.h>
#include <vtkCellDataToPointData.h>
#include <vtkCubeAxesActor.h>
#include <vtkDoubleArray.h>
#include <vtkEventQtSlotConnect.h>
#include <vtkFlyingEdges3D.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkImageActor.h>
#include <vtkImageData.h>
#include <vtkMaptkImageDataGeometryFilter.h>
#include <vtkMatrix4x4.h>
#include <vtkMetaImageWriter.h>
#include <vtkMetaImageReader.h>
#include <vtkNew.h>
#include <vtkOBJWriter.h>
#include <vtkPLYReader.h>
#include <vtkPLYWriter.h>
#include <vtkPlaneSource.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkStructuredGrid.h>
#include <vtkTextProperty.h>
#include <vtkThreshold.h>
#include <vtkTimeStamp.h>
#include <vtkUnsignedCharArray.h>
#include <vtkUnsignedIntArray.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkXMLImageDataReader.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkXMLPolyDataWriter.h>

#ifdef VTKWEBGLEXPORTER
#include <vtkScalarsToColors.h>
#include <vtkWebGLExporter.h>
#endif

#include <qtIndexRange.h>
#include <qtMath.h>
#include <qtStlUtil.h>

#include <QDebug>
#include <QFileInfo>
#include <QGuiApplication>
#include <QMenu>
#include <QTimer>
#include <QToolButton>
#include <QWidgetAction>

#include <array>

using namespace LandmarkArrays;

QTE_IMPLEMENT_D_FUNC(WorldView)

namespace // anonymous
{

//-----------------------------------------------------------------------------
vtkDataArray* findArrayToColor(vtkDataSet* dataset)
{
  bool mapScalars;
  vtkPointData* pd = dataset->GetPointData();
  // first try the active scalars
  vtkDataArray* a = pd->GetScalars();
  if (VolumeOptions::isArrayValidForColoring(a, mapScalars))
  {
    return a;
  }
  // if active scalars are not set look through all arrays,
  // if we find an array that fits, set it as active scalars and return it.
  for(int i = 0; i < pd->GetNumberOfArrays(); ++i)
  {
    a = pd->GetArray(i);
    if (VolumeOptions::isArrayValidForColoring(a, mapScalars))
    {
      pd->SetScalars(a);
      return a;
    }
    else
    {
      continue;
    }
  }
  mapScalars = false;
  return nullptr;
}

} // namespace <anonymous>

//-----------------------------------------------------------------------------
class WorldViewPrivate
{
public:
  WorldViewPrivate()
   :  initroi(false),
      rangeUpdateNeeded(false),
      validDepthInput(false),
      validImage(false),
      validTransform(false),
      cameraRepDirty(false),
      scaleDirty(false),
      axesDirty(false),
      axesVisible(false),
      renderQueued(false),
      logger(kwiver::vital::get_logger("telesculptor.worldview"))
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

  void setRobustROI();
  bool computeRobustROI(double bounds[6]);

  void
  vtkToPointList(vtkSmartPointer<vtkPolyData> mesh,
                 std::string const& colorArrayName,
                 std::vector<kwiver::vital::vector_3d>& points,
                 std::vector<kwiver::vital::rgb_color>& colors);

  Ui::WorldView UI;
  Am::WorldView AM;

  vtkNew<vtkRenderer> renderer;
  vtkSmartPointer<vtkRenderWindow> renderWindow;

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
  GroundControlPointsWidget* groundControlPointsWidget;
  RulerWidget* rulerWidget;

  VolumeOptions* volumeOptions;
  vtkNew<vtkFlyingEdges3D> contourFilter;

  vtkNew<vtkMatrix4x4> imageProjection;
  vtkNew<vtkMatrix4x4> imageLocalTransform;

  vtkSmartPointer<vtkMaptkImageDataGeometryFilter> inputDepthGeometryFilter;
  vtkNew<vtkMaptkScalarDataFilter> depthScalarFilter;
  vtkNew<vtkActor> depthMapActor;

  vtkNew<vtkActor> volumeActor;
  vtkSmartPointer<vtkImageData> volume;
  vtkSmartPointer<vtkPolyData> mesh;

  vtkSmartPointer<vtkBoxWidget2> boxWidget;
  vtkSmartPointer<vtkBox> roi;
  bool initroi;
  vtkNew<vtkEventQtSlotConnect> connections;

  bool rangeUpdateNeeded;
  bool validDepthInput;
  bool validImage;
  bool validTransform;

  bool cameraRepDirty;
  bool scaleDirty;
  bool axesDirty;

  bool axesVisible;

  bool renderQueued;

  kwiver::vital::logger_handle_t logger;
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
}

//-----------------------------------------------------------------------------
void WorldViewPrivate::updateImageTransform()
{
  vtkNew<vtkMatrix4x4> xf;

  vtkMatrix4x4::Multiply4x4(
    this->imageProjection,
    this->imageLocalTransform,
    xf);

  this->imageActor->SetUserMatrix(xf);
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
void WorldViewPrivate::setRobustROI()
{
  if (this->roi)
  {
    double bounds[6];
    if (this->computeRobustROI(bounds))
    {
      this->roi->SetBounds(bounds);
    }
  }
}

//-----------------------------------------------------------------------------
bool WorldViewPrivate::computeRobustROI(double bounds[6])
{
  constexpr double percentile = 0.1;
  // use a different percentile for z-max, that's typically where you have
  // small structures (poles, towers) with few points.
  constexpr double zmax_percentile = 0.01;
  constexpr double margin = 0.5;
  this->landmarkActor->GetMapper()->Update();
  vtkIdType numPts = this->landmarkPoints->GetNumberOfPoints();
  if (numPts < 2)
  {
    return false;
  }
  std::vector<double> x, y, z;
  x.reserve(numPts);
  y.reserve(numPts);
  z.reserve(numPts);
  for (vtkIdType i = 0; i < numPts; ++i)
  {
    double pt[3];
    this->landmarkPoints->GetPoint(i, pt);
    x.push_back(pt[0]);
    y.push_back(pt[1]);
    z.push_back(pt[2]);
  }
  std::sort(x.begin(), x.end());
  std::sort(y.begin(), y.end());
  std::sort(z.begin(), z.end());
  vtkIdType minIdx = static_cast<vtkIdType>(percentile * (numPts - 1));
  vtkIdType maxIdx = static_cast<vtkIdType>(numPts - 1 - minIdx);
  vtkIdType zmaxIdx = static_cast<vtkIdType>((numPts - 1) *
                                              (1.0 - zmax_percentile));
  bounds[0] = x[minIdx];
  bounds[1] = x[maxIdx];
  bounds[2] = y[minIdx];
  bounds[3] = y[maxIdx];
  bounds[4] = z[minIdx];
  bounds[5] = z[zmaxIdx];
  for (unsigned i = 0; i < 3; ++i)
  {
    unsigned i_min = 2 * i;
    unsigned i_max = i_min + 1;
    double offset = (bounds[i_max] - bounds[i_min]) * margin;
    bounds[i_min] -= offset;
    bounds[i_max] += offset;
  }
  return true;
}

//-----------------------------------------------------------------------------
void
WorldViewPrivate::vtkToPointList(vtkSmartPointer<vtkPolyData> data,
                                 std::string const& colorArrayName,
                                 std::vector<kwiver::vital::vector_3d>& points,
                                 std::vector<kwiver::vital::rgb_color>& colors)
{
  namespace kv = kwiver::vital;
  vtkPoints *inPts = data->GetPoints();
  vtkIdType numPts = inPts->GetNumberOfPoints();
  points.resize(numPts);
  colors.clear();

  vtkDataArray* da = nullptr;
  if (this->volumeOptions->isColorOptionsEnabled())
  {
    da = data->GetPointData()->GetArray(colorArrayName.c_str());
  }
  vtkUnsignedCharArray* rgbArray = nullptr;
  if (da != nullptr && da->GetNumberOfComponents() == 3)
  {
    rgbArray = vtkArrayDownCast<vtkUnsignedCharArray>(da);
    colors.resize(numPts);
  }

  for (vtkIdType i = 0; i < numPts; ++i)
  {
    inPts->GetPoint(i, points[i].data());
    if (rgbArray)
    {
      vtkIdType idx = 3 * i;
      colors[i] = kv::rgb_color(rgbArray->GetValue(idx),
                                rgbArray->GetValue(idx + 1),
                                rgbArray->GetValue(idx + 2));
    }
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

  d->renderWindow =
    vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();

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
  d->imageOptions->addActor(d->imageActor);
  d->setPopup(d->UI.actionShowFrameImage, d->imageOptions);

  connect(d->imageOptions, &ImageOptions::modified,
          this, &WorldView::render);

  d->cameraOptions = new CameraOptions{d->cameraRep, this};
  d->setPopup(d->UI.actionShowCameras, d->cameraOptions);

  connect(d->cameraOptions, &CameraOptions::modified,
          this, &WorldView::invalidateGeometry);

  d->landmarkOptions = new PointOptions("WorldView/Landmarks", this);
  d->landmarkOptions->addActor(d->landmarkActor);
  d->setPopup(d->UI.actionShowLandmarks, d->landmarkOptions);

  connect(d->landmarkOptions, &PointOptions::modified,
          this, &WorldView::render);

  d->depthMapOptions = new DepthMapOptions("WorldView/DepthMap", this);
  d->setPopup(d->UI.actionShowDepthMap, d->depthMapOptions);

  d->depthMapOptions->setEnabled(false);

  connect(d->depthMapOptions, &DepthMapOptions::displayModeChanged,
          this, &WorldView::updateDepthMapDisplayMode);
  connect(d->depthMapOptions,
          QOverload<bool>::of(&DepthMapOptions::thresholdsChanged),
          this, &WorldView::updateDepthMapThresholds);

  d->volumeOptions = new VolumeOptions("WorldView/Volume", this);
  d->setPopup(d->UI.actionShowVolume, d->volumeOptions);

  d->volumeOptions->setEnabled(false);

  connect(d->volumeOptions, &VolumeOptions::modified,
          this, &WorldView::render);

  connect(this, &WorldView::contourChanged,
          d->volumeOptions, &VolumeOptions::reshowColorizeSurfaceMenu);

  auto const roiMenu = new QMenu(this);
  roiMenu->addAction(d->UI.actionResetROI);
  d->UI.actionResetROI->setDisabled(true);
  d->setPopup(d->UI.actionSelectROI, roiMenu);

  // Connect actions
  this->addAction(d->UI.actionViewReset);
  this->addAction(d->UI.actionViewResetLandmarks);
  this->addAction(d->UI.actionViewPerspective);
  this->addAction(d->UI.actionShowCameras);
  this->addAction(d->UI.actionShowLandmarks);
  this->addAction(d->UI.actionShowGroundPlane);
  this->addAction(d->UI.actionShowDepthMap);
  this->addAction(d->UI.actionShowVolume);
  this->addAction(d->UI.actionPlaceEditGCP);

  connect(d->UI.actionViewReset, &QAction::triggered,
          this, &WorldView::resetView);
  connect(d->UI.actionViewResetLandmarks, &QAction::triggered,
          this, &WorldView::resetViewToLandmarks);
  connect(d->UI.actionViewToWorldTop, &QAction::triggered,
          this, &WorldView::viewToWorldTop);
  connect(d->UI.actionViewToWorldLeft, &QAction::triggered,
          this, &WorldView::viewToWorldLeft);
  connect(d->UI.actionViewToWorldRight, &QAction::triggered,
          this, &WorldView::viewToWorldRight);
  connect(d->UI.actionViewToWorldFront, &QAction::triggered,
          this, &WorldView::viewToWorldFront);
  connect(d->UI.actionViewToWorldBack, &QAction::triggered,
          this, &WorldView::viewToWorldBack);
  connect(d->UI.actionViewPerspective, &QAction::toggled,
          this, &WorldView::setPerspective);

  connect(d->UI.actionShowFrameImage, &QAction::toggled,
          this, &WorldView::setImageVisible);
  connect(d->UI.actionShowCameras, &QAction::toggled,
          this, &WorldView::setCamerasVisible);
  connect(d->UI.actionShowLandmarks, &QAction::toggled,
          this, &WorldView::setLandmarksVisible);
  connect(d->UI.actionShowGroundPlane, &QAction::toggled,
          this, &WorldView::setGroundPlaneVisible);
  connect(d->UI.actionShowDepthMap, &QAction::toggled,
          this, &WorldView::setDepthMapVisible);
  connect(d->UI.actionShowDepthMap, &QAction::toggled,
          this, &WorldView::depthMapEnabled);
  connect(d->UI.actionSelectROI, &QAction::toggled,
          this, &WorldView::selectROI);
  connect(d->UI.actionResetROI, &QAction::triggered,
          this, &WorldView::resetROI);

  connect(d->UI.actionShowVolume, &QAction::toggled,
          this, &WorldView::setVolumeVisible);

  // Set up render pipeline
  d->renderer->SetBackground(0, 0, 0);
  d->renderWindow->AddRenderer(d->renderer);
#if VTK_VERSION_MAJOR < 9
  d->UI.renderWidget->SetRenderWindow(d->renderWindow);
  auto renderInteractor = d->UI.renderWidget->GetInteractor();
#else
  d->UI.renderWidget->setRenderWindow(d->renderWindow);
  auto renderInteractor = d->UI.renderWidget->interactor();
#endif
  d->groundControlPointsWidget = new GroundControlPointsWidget(this);
  d->groundControlPointsWidget->setInteractor(renderInteractor);
  connect(d->UI.actionPlaceEditGCP, &QAction::toggled,
          this, &WorldView::pointPlacementEnabled);

  d->rulerWidget = new RulerWidget(this);
  d->rulerWidget->setInteractor(renderInteractor);
  connect(d->UI.actionShowRuler, &QAction::toggled,
          this, &WorldView::rulerEnabled);

  vtkNew<vtkMaptkInteractorStyle> iren;
  d->renderWindow->GetInteractor()->SetInteractorStyle(iren);

  d->renderer->AddActor(d->cameraRep->GetNonActiveActor());
  d->renderer->AddActor(d->cameraRep->GetActiveActor());
  d->renderer->AddActor(d->cameraRep->GetPathActor());

  // Set up image actor and "dummy" data for use when we have no "real" image
  d->imageActor->SetVisibility(false);
  d->imageActor->PickableOff();
  d->renderer->AddViewProp(d->imageActor);

  // Enable antialiasing by default
  d->renderer->UseFXAAOn();

  d->emptyImage->SetExtent(0, 0, 0, 0, 0, 0);
  d->emptyImage->AllocateScalars(VTK_UNSIGNED_CHAR, 1);
  d->emptyImage->SetScalarComponentFromDouble(0, 0, 0, 0, 0.0);

  this->setImageData(nullptr, QSize{1, 1});

  // Set up landmark actor
  vtkNew<vtkPolyData> landmarkPolyData;

  auto const landmarkPointData = landmarkPolyData->GetPointData();

  d->landmarkColors->SetName(TrueColor);
  d->landmarkColors->SetNumberOfComponents(3);

  d->landmarkElevations->SetName(Elevation);
  d->landmarkElevations->SetNumberOfComponents(1);

  d->landmarkObservations->SetName(Observations);
  d->landmarkObservations->SetNumberOfComponents(1);

  landmarkPolyData->SetPoints(d->landmarkPoints);
  landmarkPolyData->SetVerts(d->landmarkVerts);
  landmarkPointData->AddArray(d->landmarkColors);
  landmarkPointData->AddArray(d->landmarkElevations);
  landmarkPointData->AddArray(d->landmarkObservations);
  d->landmarkMapper->SetInputData(landmarkPolyData);

  d->landmarkActor->SetMapper(d->landmarkMapper);
  d->landmarkActor->SetVisibility(d->UI.actionShowLandmarks->isChecked());
  d->renderer->AddActor(d->landmarkActor);

  d->landmarkOptions->addMapper(d->landmarkMapper);

  // Set up ground plane grid
  d->groundPlane->SetOrigin(-10.0, -10.0, 0.0);
  d->groundPlane->SetPoint1(+10.0, -10.0, 0.0);
  d->groundPlane->SetPoint2(-10.0, +10.0, 0.0);
  d->groundPlane->SetResolution(20, 20);

  vtkNew<vtkPolyDataMapper> groundMapper;
  groundMapper->SetInputConnection(d->groundPlane->GetOutputPort());
  groundMapper->SetResolveCoincidentTopologyToPolygonOffset();

  d->groundActor->SetMapper(groundMapper);
  d->groundActor->GetProperty()->SetColor(0.5, 0.5, 0.5);
  d->groundActor->GetProperty()->SetLighting(false);
  d->groundActor->GetProperty()->SetRepresentationToWireframe();
  d->groundActor->PickableOff();
  d->renderer->AddActor(d->groundActor);

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
  d->cubeAxesActor->SetGridLineLocation(
    vtkCubeAxesActor::VTK_GRID_LINES_FURTHEST);

  d->cubeAxesActor->SetCamera(d->renderer->GetActiveCamera());
  d->cubeAxesActor->SetVisibility(false);
  d->cubeAxesActor->PickableOff();

  d->renderer->AddActor(d->cubeAxesActor);

  // Setup DepthMap actor
  d->depthScalarFilter->SetScalarArrayName(DepthMapArrays::TrueColor);
  vtkNew<vtkPolyDataMapper> mapper;
  mapper->SetInputConnection(d->depthScalarFilter->GetOutputPort());
  mapper->SetColorModeToDirectScalars();
  d->depthMapActor->SetMapper(mapper);
  d->renderer->AddActor(d->depthMapActor);
  d->depthMapActor->VisibilityOff();

  // Add keyboard actions for increasing and descreasing depth point size
  QAction* actionIncreasePointSize = new QAction(this);
  actionIncreasePointSize->setShortcut(Qt::Key_Plus);
  actionIncreasePointSize->setShortcutContext(Qt::WidgetWithChildrenShortcut);
  d->UI.renderWidget->addAction(actionIncreasePointSize);
  connect(actionIncreasePointSize, &QAction::triggered,
          this, &WorldView::increaseDepthMapPointSize);

  QAction* actionDecreasePointSize = new QAction(this);
  actionDecreasePointSize->setShortcut(Qt::Key_Minus);
  actionDecreasePointSize->setShortcutContext(Qt::WidgetWithChildrenShortcut);
  d->UI.renderWidget->addAction(actionDecreasePointSize);
  connect(actionDecreasePointSize, &QAction::triggered,
          this, &WorldView::decreaseDepthMapPointSize);
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
  this->render();
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
    d->depthScalarFilter->SetInputConnection(nullptr);
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
void WorldView::setDepthGeometryFilter(
  vtkMaptkImageDataGeometryFilter* geometryFilter)
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

  this->render();
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
    auto const bcArray = pd->GetArray(DepthMapArrays::Weight);
    auto const urArray = pd->GetArray(DepthMapArrays::Uncertainty);

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
void WorldView::setVideoConfig(QString const& path,
                               kwiver::vital::config_block_sptr config)
{
  QTE_D();

  d->volumeOptions->setVideoConfig(stdString(path), config);
}

//-----------------------------------------------------------------------------
void WorldView::setMaskConfig(QString const& path,
                               kwiver::vital::config_block_sptr config)
{
  QTE_D();

  d->volumeOptions->setMaskConfig(path.toStdString(), config);
}

//-----------------------------------------------------------------------------
void WorldView::invalidateCameras()
{
  QTE_D();

  d->cameraRep->CamerasModified();
}

//-----------------------------------------------------------------------------
bool WorldView::loadVolume(QString const& path)
{
  QFileInfo fileInfo{path};
  if (!fileInfo.exists() || !fileInfo.isFile())
  {
    return false;
  }

  // Create the vtk pipeline and read the volume
  vtkNew<vtkMetaImageReader> reader;
  reader->SetFileName(qPrintable(path));
  reader->Update();

  auto volume = vtkImageData::SafeDownCast(reader->GetOutput());
  if (volume->GetPointData()->GetNumberOfArrays() <= 0)
  {
    return false;
  }

  volume->GetPointData()->GetAbstractArray(0)->SetName("reconstruction_scalar");
  this->setVolume(volume);

  return true;
}

//-----------------------------------------------------------------------------
//TODO: add camera and video for coloring
void WorldView::setVolume(vtkSmartPointer<vtkImageData> volume)
{
  QTE_D();

  d->UI.actionShowVolume->setEnabled(true);

  d->volume = volume;

  // Apply contour
  d->contourFilter->SetInputData(volume);
  d->contourFilter->SetNumberOfContours(1);
  d->contourFilter->SetValue(0, 0.0);
  // Declare which table will be use for the contour
  d->contourFilter->SetInputArrayToProcess(0, 0, 0,
    vtkDataObject::FIELD_ASSOCIATION_POINTS,
    "reconstruction_scalar");

  d->contourFilter->Update();
  d->mesh = d->contourFilter->GetOutput();

  // Create mapper
  vtkNew<vtkPolyDataMapper> contourMapper;
  contourMapper->SetInputConnection(d->contourFilter->GetOutputPort());
  contourMapper->ScalarVisibilityOff();

  // Set the actor's mapper
  d->volumeActor->SetMapper(contourMapper);
  d->volumeActor->GetProperty()->SetColor(0.7, 0.7, 0.7);
  d->volumeActor->GetProperty()->SetAmbient(0.25);
  d->volumeActor->GetProperty()->SetDiffuse(0.75);
  d->volumeActor->SetVisibility(true);
  d->volumeOptions->setActor(d->volumeActor);
  d->volumeOptions->setEnabled(true);
  d->volumeOptions->setSurfaceColoringMode(VolumeOptions::NO_COLOR);
  d->volumeOptions->disableOriginalColor();

  emit this->volumeEnabled(true);
  emit this->fusedMeshEnabled(true);

  this->setVolumeVisible(d->UI.actionShowVolume->isChecked());

  // Add this actor to the renderer
  d->renderer->AddActor(d->volumeActor);
  emit contourChanged();
}

//-----------------------------------------------------------------------------
void WorldView::resetVolume()
{
  QTE_D();

  d->volume = nullptr;
  d->mesh = nullptr;

  d->renderer->RemoveActor(d->volumeActor);

  emit this->volumeEnabled(false);
  emit this->fusedMeshEnabled(false);
  d->volumeOptions->setEnabled(false);
}

//-----------------------------------------------------------------------------
void WorldView::setVolumeVisible(bool state)
{
  QTE_D();

  if (d->volume || d->mesh)
  {
    if (static_cast<bool>(d->volumeActor->GetVisibility()) != state)
    {
      d->volumeActor->SetVisibility(state);
      d->volumeOptions->setEnabled(state);
      this->render();

      if (d->volume)
      {
        emit this->volumeEnabled(state);
      }
      if (d->mesh)
      {
        emit this->fusedMeshEnabled(state);
      }
    }
  }
}

//-----------------------------------------------------------------------------
void WorldView::setVolumeCurrentFrame(kwiver::vital::frame_id_t frame)
{
  QTE_D();

  d->volumeOptions->setCurrentFrame(frame);
}

//-----------------------------------------------------------------------------
void WorldView::computeContour(double threshold)
{
  QTE_D();

  d->contourFilter->SetValue(0, threshold);
  d->contourFilter->Update();
  if (d->volumeOptions->isColorOptionsEnabled())
  {
    d->volumeOptions->forceColorize();
  }
  vtkSmartPointer<vtkPolyData> mesh = d->contourFilter->GetOutput();
  if (mesh)
  {
    d->mesh = mesh;
  }
}

//-----------------------------------------------------------------------------
bool WorldView::loadMesh(QString const& path)
{
  QTE_D();

  QFileInfo fileInfo{path};
  if (!fileInfo.exists() || !fileInfo.isFile())
  {
    return false;
  }

  QGuiApplication::setOverrideCursor(Qt::WaitCursor);
  vtkSmartPointer<vtkPolyData> mesh;
  QString const& ext = fileInfo.suffix().toLower();
  if (ext == "ply")
  {
    vtkNew<vtkPLYReader> reader;
    reader->SetFileName(qPrintable(path));
    reader->Update();
    mesh = vtkPolyData::SafeDownCast(reader->GetOutput());
  }
  else if (ext == "vtp")
  {
    vtkNew<vtkXMLPolyDataReader> reader;
    reader->SetFileName(qPrintable(path));
    reader->Update();
    mesh = vtkPolyData::SafeDownCast(reader->GetOutput());
  }
  else
  {
    LOG_ERROR(d->logger, "Invalid mesh type: " << qPrintable(fileInfo.suffix()));
    return false;
  }
  QGuiApplication::restoreOverrideCursor();

  this->setMesh(mesh);

  return true;
}

//-----------------------------------------------------------------------------
void WorldView::setMesh(vtkSmartPointer<vtkPolyData> mesh)
{
  QTE_D();

  if (mesh->GetNumberOfCells() == 0)
  {
    vtkNew<vtkVertexGlyphFilter> vertexFilter;
    vertexFilter->SetInputData(mesh);
    vertexFilter->Update();
    mesh = vertexFilter->GetOutput();
  }

  d->mesh = mesh;

  // Create mapper
  vtkNew<vtkPolyDataMapper> meshMapper;
  meshMapper->SetInputData(mesh);

  // Set the actor's mapper
  d->volumeActor->SetMapper(meshMapper);
  d->volumeActor->SetVisibility(true);
  // Add this actor to the renderer
  d->renderer->AddActor(d->volumeActor);
  emit this->fusedMeshEnabled(true);
  d->volumeOptions->setActor(d->volumeActor);
  d->volumeOptions->setEnabled(true);

  this->setVolumeVisible(d->UI.actionShowVolume->isChecked());
  std::array<std::string, 3> requiredArrays = {"mean", "median", "count"};
  size_t i;
  for (i = 0; i < requiredArrays.size(); ++i)
  {
    if (! mesh->GetPointData()->GetArray(requiredArrays[i].c_str()))
    {
      break;
    }
  }
  if (i == requiredArrays.size())
  {
    // if the mesh has the required arrays set IMAGE_COLOR and
    // avoid recomputing colors from current image.
    d->volumeOptions->setSurfaceColoringMode(
      VolumeOptions::IMAGE_COLOR, true /*blockSignals*/);
  }
  else
  {
    // check if there is other scalar to color
    vtkDataArray* scalars = findArrayToColor(mesh);
    if (scalars)
    {
      d->volumeOptions->setOriginalColorArray(scalars);
      // make sure refresh the picture even when setting the same ORIGINAL_COLOR
      d->volumeOptions->setSurfaceColoringMode(
        VolumeOptions::ORIGINAL_COLOR, true /*blockSignals*/);
      d->volumeOptions->surfaceColoringModeChanged(
        VolumeOptions::ORIGINAL_COLOR);
    }
    else
    {
      d->volumeOptions->setSurfaceColoringMode(VolumeOptions::NO_COLOR);
    }
  }
}

//-----------------------------------------------------------------------------
void WorldView::addCamera(
  kwiver::vital::frame_id_t id, kwiver::arrows::vtk::vtkKwiverCamera* camera)
{
  QTE_D();

  d->cameraRep->AddCamera(id, camera);

  d->volumeOptions->setCamera(id, camera->GetCamera());

  d->updateCameras(this);
  d->updateAxes(this);
}

//-----------------------------------------------------------------------------
void WorldView::updateCamera(
  kwiver::vital::frame_id_t id, kwiver::arrows::vtk::vtkKwiverCamera* camera)
{
  QTE_D();

  d->volumeOptions->setCamera(id, camera->GetCamera());

  d->updateCameras(this);
  d->updateAxes(this);
}

//-----------------------------------------------------------------------------
void WorldView::removeCamera(kwiver::vital::frame_id_t id)
{
  QTE_D();

  d->cameraRep->RemoveCamera(id);

  d->volumeOptions->setCamera(id, nullptr);

  d->updateCameras(this);
  d->updateAxes(this);
}

//-----------------------------------------------------------------------------
void WorldView::setActiveCamera(kwiver::vital::frame_id_t id)
{
  static auto const plane = kwiver::vital::vector_4d(0.0, 0.0, 1.0, 0.0);

  QTE_D();

  d->cameraRep->SetActiveCamera(id);
  auto* const camera =
    dynamic_cast<kwiver::arrows::vtk::vtkKwiverCamera*>(d->cameraRep->GetActiveCamera());

  if (camera)
  {
    camera->GetTransform(d->imageProjection, plane.data());
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
void WorldView::setImageData(vtkImageData* data, QSize dimensions)
{
  QTE_D();

  d->imageLocalTransform->Identity();
  d->imageLocalTransform->SetElement(1, 1, -1.0);
  d->imageLocalTransform->SetElement(1, 3, dimensions.height());
  d->updateImageTransform();

  auto const showImage = d->UI.actionShowFrameImage->isChecked();
  d->validImage = data;
  d->imageActor->SetInputData(data ? data : d->emptyImage);
  d->imageActor->SetVisibility(data && d->validTransform && showImage);
  this->render();
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
  auto autoMinZ = qInf(), autoMaxZ = -qInf();
  std::vector<double> zValues;

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
  foreach (auto const& landmark, landmarks)
  {
    auto const& pos = landmark.second->loc();
    auto const& color = landmark.second->color();
    auto const observations = landmark.second->observations();

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
    zValues.push_back(pos[2]);
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
                                                   { 0.0, upper }});
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
    this->render();
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
    this->render();
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
  this->render();
}

//-----------------------------------------------------------------------------
void WorldView::resetViewToLandmarks()
{
  QTE_D();

  vtkBoundingBox bbox;
  bbox.AddBounds(d->landmarkActor->GetBounds());

  if (!bbox.IsValid())
  {
    return;
  }

  double bounds[6];
  bbox.GetBounds(bounds);
  d->renderer->ResetCamera(bounds);
  d->renderer->ResetCameraClippingRange();

  this->render();
}

//-----------------------------------------------------------------------------
void WorldView::viewToWorldTop()
{
  QTE_D();
  d->setView(0.0, 0.0, +1.0, 0.0, +1.0, 0.0);
  this->render();
}

//-----------------------------------------------------------------------------
void WorldView::viewToWorldLeft()
{
  QTE_D();
  d->setView(-1.0, 0.0, 0.0, 0.0, 0.0, +1.0);
  this->render();
}

//-----------------------------------------------------------------------------
void WorldView::viewToWorldRight()
{
  QTE_D();
  d->setView(+1.0, 0.0, 0.0, 0.0, 0.0, +1.0);
  this->render();
}

//-----------------------------------------------------------------------------
void WorldView::viewToWorldFront()
{
  QTE_D();
  d->setView(0.0, -1.0, 0.0, 0.0, 0.0, +1.0);
  this->render();
}

//-----------------------------------------------------------------------------
void WorldView::viewToWorldBack()
{
  QTE_D();
  d->setView(0.0, +1.0, 0.0, 0.0, 0.0, +1.0);
  this->render();
}

//-----------------------------------------------------------------------------
void WorldView::setPerspective(bool perspective)
{
  QTE_D();

  d->renderer->GetActiveCamera()->SetParallelProjection(!perspective);
  this->render();
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
    this->render();
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
      if (prop == d->cubeAxesActor ||
          prop == d->groundActor ||
          prop == d->cameraRep->GetActiveActor() ||
          prop == d->cameraRep->GetNonActiveActor() ||
          !prop->GetVisibility() ||
          !prop->GetBounds())
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
    this->render();
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

    // Add ROI bounds (if ROI set) otherwise estimate robust ROI and use that
    double bounds[6];
    d->roi->GetBounds(bounds);
    if ( ( bounds[1] >= bounds[0] &&
           bounds[3] >= bounds[2] &&
           bounds[5] >= bounds[4] ) ||
         d->computeRobustROI(bounds))
    {
      bbox.AddBounds(bounds);
    }

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
      // Compute base scale
      auto const cameraScale = 0.9 * bbox.GetDiagonalLength();

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
  this->connectDepthPipeline();
  this->render();
}

//-----------------------------------------------------------------------------
void WorldView::updateDepthMapThresholds(bool filterState)
{
  QTE_D();

  double weightMin = d->depthMapOptions->weightMinimum();
  double weightMax = d->depthMapOptions->weightMaximum();
  double uncertaintyMin = d->depthMapOptions->uncertaintyMinimum();
  double uncertaintyMax = d->depthMapOptions->uncertaintyMaximum();

  d->inputDepthGeometryFilter->SetConstraint(
    DepthMapArrays::Weight, weightMin, weightMax);
  d->inputDepthGeometryFilter->SetConstraint(
    DepthMapArrays::Uncertainty, uncertaintyMin, uncertaintyMax);
  d->inputDepthGeometryFilter->SetThresholdCells(filterState);

  emit depthMapThresholdsChanged();

  this->render();
}

//-----------------------------------------------------------------------------
bool WorldView::saveDepthPoints(QString const& path,
                                kwiver::vital::local_geo_cs const& lgcs)
{
  QTE_D();
  namespace kv = kwiver::vital;
  if (QFileInfo(path).suffix().toLower() == "las")
  {
    // convert the point cloud into a set of landmarks,
    // then use the PDAL writer for landmarks
    vtkSmartPointer<vtkPolyData> data =
      vtkPolyData::SafeDownCast(d->depthScalarFilter->GetOutput());
    std::vector<kv::vector_3d> points;
    std::vector<kv::rgb_color> colors;
    d->vtkToPointList(data, DepthMapArrays::TrueColor, points, colors);

    kv::algo::pointcloud_io_sptr pc_io = kv::create_algorithm<kv::algo::pointcloud_io>("pdal");
    if(!pc_io)
    {
      LOG_ERROR(d->logger,
        "Unable to write file: failed to create PDAL pointcloud writer");
      return false;
    }
    pc_io->set_local_geo_cs(lgcs);
    pc_io->save(stdString(path), points, colors);
    return true;
  }
  else
  {
    vtkNew<vtkPLYWriter> writer;

    writer->SetFileName(qPrintable(path));
    writer->SetInputConnection(d->depthScalarFilter->GetOutputPort());
    writer->SetColorMode(0);
    writer->SetArrayName(DepthMapArrays::TrueColor);
    auto const result = writer->Write();
    return (result == VTK_OK);
  }
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
bool WorldView::saveVolume(const QString &path)
{
  QTE_D();

  //NOTE: For now, the volume is set in the configuration parameters.
  //      It may be generated directly from the GUI in the future.

  vtkNew<vtkMetaImageWriter> mIWriter;
  mIWriter->SetFileName(qPrintable(path));
  mIWriter->SetInputData(d->volume);
  mIWriter->SetCompression(true);
  mIWriter->Write();

  LOG_INFO(d->logger, "Saved : " << qPrintable(path));
  return true;
}

//-----------------------------------------------------------------------------
bool WorldView::saveFusedMesh(const QString &path,
                              kwiver::vital::local_geo_cs const& lgcs)
{
  QTE_D();
  namespace kv = kwiver::vital;

  if (!d->mesh)
  {
    LOG_ERROR(d->logger, "No mesh available to save");
    return false;
  }

  int result;
  const QString ext = QFileInfo(path).suffix().toLower();
  if (ext == "ply")
  {
    vtkNew<vtkPLYWriter> writer;
    writer->SetFileName(qPrintable(path));
    writer->SetColorMode(0);
    if (d->volumeOptions->isColorOptionsEnabled())
    {
      writer->SetArrayName(d->mesh->GetPointData()->GetScalars()->GetName());
      writer->SetLookupTable(d->volumeActor->GetMapper()->GetLookupTable());
    }
    writer->AddInputDataObject(d->mesh);
    result = writer->Write();
  }
  else if (ext == "obj")
  {
    vtkNew<vtkOBJWriter> writer;
    writer->SetFileName(qPrintable(path));
    writer->AddInputDataObject(d->mesh);
    result = writer->Write();
  }
  else if (ext == "las")
  {
    std::vector<kv::vector_3d> points;
    std::vector<kv::rgb_color> colors;
    d->vtkToPointList(d->mesh, d->mesh->GetPointData()->GetScalars()->GetName(),
                      points, colors);
    kv::algo::pointcloud_io_sptr pc_io = kv::create_algorithm<kv::algo::pointcloud_io>("pdal");
    if(!pc_io)
    {
      LOG_ERROR(d->logger,
        "Unable to write file: failed to create PDAL pointcloud writer");
      return false;
    }
    pc_io->set_local_geo_cs(lgcs);
    pc_io->save(stdString(path), points, colors);
    result = VTK_OK;
  }
  else
  {
    vtkNew<vtkXMLPolyDataWriter> writer;
    writer->SetFileName(qPrintable(path));
    writer->SetDataModeToBinary();
    writer->AddInputDataObject(d->mesh);
    result = writer->Write();
  }

  LOG_INFO(d->logger, "Saved : " << qPrintable(path));
  return (result == VTK_OK);
}

//-----------------------------------------------------------------------------
void WorldView::saveFusedMeshFrameColors(const QString &path, bool occlusion)
{
  QTE_D();

  if (!d->mesh)
  {
    LOG_ERROR(d->logger, "No mesh available to save");
    return;
  }

  QEventLoop loop;
  std::string videoPath = d->volumeOptions->getVideoPath();
  kwiver::vital::config_block_sptr videoConfig =  d->volumeOptions->getVideoConfig();
  if (!videoConfig || videoPath.empty())
  {
    LOG_ERROR(d->logger, "Unable to extract colors, no video loaded");
    return;
  }

  std::string maskPath = d->volumeOptions->getMaskPath();
  kwiver::vital::config_block_sptr maskConfig =  d->volumeOptions->getMaskConfig();
  kwiver::vital::camera_map_sptr cameras = d->volumeOptions->getCameras();
  if (!cameras || cameras->size() == 0)
  {
    LOG_ERROR(d->logger, "Unable to extract colors, no camera models available");
    return;
  }

  MeshColoration* coloration =
    new MeshColoration(videoConfig, videoPath,
                       maskConfig, maskPath,
                       cameras);
  coloration->setProperty("mesh_output_path", path);
  coloration->set_input(d->mesh);
  coloration->set_frame_sampling(d->volumeOptions->getFrameSampling());
  double occlusionThreshold = d->volumeOptions->getOcclusionThreshold();
  coloration->set_occlusion_threshold(occlusionThreshold);
  coloration->set_color_occluded(!occlusion);
  coloration->set_frame(-1);
  coloration->set_all_frames(true);
  connect(coloration, &MeshColoration::resultReady,
          this, &WorldView::meshColorationHandleResult);
  connect(coloration, &MeshColoration::resultReady, &loop, &QEventLoop::quit);
  connect(coloration, &MeshColoration::finished,
          coloration, &MeshColoration::deleteLater);
  coloration->start();
  loop.exec();
}

//-----------------------------------------------------------------------------
void WorldView::meshColorationHandleResult(MeshColoration* coloration)
{
  QTE_D();

  if (coloration)
  {
    QString path = coloration->property("mesh_output_path").toString();
    vtkNew<vtkXMLPolyDataWriter> writer;
    writer->SetFileName(qPrintable(path));
    writer->SetDataModeToBinary();
    writer->AddInputDataObject(coloration->get_output());
    writer->Write();

    LOG_INFO(d->logger, "Saved : " << qPrintable(path));
  }
}

//-----------------------------------------------------------------------------
void WorldView::increaseDepthMapPointSize()
{
  QTE_D();

  float pointSize = d->depthMapActor->GetProperty()->GetPointSize();
  d->depthMapActor->GetProperty()->SetPointSize(pointSize + 0.5);

  this->render();
}

//-----------------------------------------------------------------------------
void WorldView::decreaseDepthMapPointSize()
{
  QTE_D();

  float pointSize = d->depthMapActor->GetProperty()->GetPointSize() - 0.5;
  d->depthMapActor->GetProperty()->SetPointSize(pointSize < 1 ? 1 : pointSize);

  this->render();
}

//-----------------------------------------------------------------------------
void WorldView::render()
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
void WorldView::enableAntiAliasing(bool enable)
{
  QTE_D();

  d->renderer->SetUseFXAA(enable);
  this->render();
}

//-----------------------------------------------------------------------------
void WorldView::selectROI(bool toggled)
{
  QTE_D();

  if (toggled && (d->landmarkPoints->GetNumberOfPoints() > 1 || d->initroi))
  {
    if (!d->boxWidget)
    {
      d->boxWidget =
        vtkSmartPointer<vtkBoxWidget2>::New();
      d->boxWidget->SetInteractor(d->renderWindow->GetInteractor());
      d->boxWidget->RotationEnabledOff();
      vtkBoxRepresentation* rep =
        vtkBoxRepresentation::SafeDownCast(d->boxWidget->GetRepresentation());
      if (rep)
      {
        rep->SetPlaceFactor(1); // Default is 0.5
        if (d->initroi)
        {
          rep->PlaceWidget(d->roi->GetBounds());
          d->initroi = false;
        }
        else
        {
          d->setRobustROI();
          rep->PlaceWidget(d->roi->GetBounds());
        }
        d->connections->Connect(
          d->boxWidget,
          vtkCommand::InteractionEvent,
          this,
          SLOT(updateROI(vtkObject*, unsigned long, void*, void*)));
      }
    }
    d->boxWidget->On();
    d->UI.actionResetROI->setEnabled(true);
  }
  else if (d->boxWidget)
  {
    d->boxWidget->Off();
    d->UI.actionResetROI->setEnabled(false);
  }
  this->render();
}

//-----------------------------------------------------------------------------
void WorldView::resetROI()
{
  QTE_D();

  if (d->roi && d->landmarkPoints->GetNumberOfPoints() > 2)
  {
    d->setRobustROI();
    if (d->boxWidget)
    {
      vtkBoxRepresentation* rep =
        vtkBoxRepresentation::SafeDownCast(d->boxWidget->GetRepresentation());
      if (rep)
      {
        rep->PlaceWidget(d->roi->GetBounds());
      }
    }
    d->updateScale(this);
    this->render();
  }
}

//-----------------------------------------------------------------------------
void WorldView::setROI(vtkBox* box, bool init)
{
  if (!box)
  {
    return;
  }

  QTE_D();
  d->roi = box;
  d->initroi = init;
  if (d->boxWidget)
  {
    vtkBoxRepresentation* rep =
      vtkBoxRepresentation::SafeDownCast(d->boxWidget->GetRepresentation());
    if (rep)
    {
      rep->PlaceWidget(d->roi->GetBounds());
    }
  }
  d->updateScale(this);
  this->render();
}

//-----------------------------------------------------------------------------
void WorldView::updateROI(vtkObject* caller,
                          unsigned long,
                          void*,
                          void*)
{
  QTE_D();

  vtkBoxWidget2* w = reinterpret_cast<vtkBoxWidget2*>(caller);
  vtkBoxRepresentation* rep =
    vtkBoxRepresentation::SafeDownCast(w->GetRepresentation());
  if (rep && d->roi)
  {
    d->roi->SetBounds(rep->GetBounds());
    d->updateScale(this);
  }
}

//-----------------------------------------------------------------------------
void WorldView::setEditMode(EditMode mode)
{
  QTE_D();

  d->groundControlPointsWidget->enableWidget(
    mode == EditMode::GroundControlPoints);
  d->UI.actionPlaceEditGCP->setChecked(
    mode == EditMode::GroundControlPoints);
}

//-----------------------------------------------------------------------------
GroundControlPointsWidget* WorldView::groundControlPointsWidget() const
{
  QTE_D();

  return d->groundControlPointsWidget;
}

//-----------------------------------------------------------------------------
RulerWidget* WorldView::rulerWidget() const
{
  QTE_D();

  return d->rulerWidget;
}

//-----------------------------------------------------------------------------
void WorldView::setRulerOptions(RulerOptions* r)
{
  QTE_D();
  d->setPopup(d->UI.actionShowRuler, r);
  connect(r, &RulerOptions::resetRuler,
          this, [=]() { d->UI.actionShowRuler->setChecked(false); });
}
