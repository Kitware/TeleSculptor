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

#include "MainWindow.h"

#include "ui_MainWindow.h"
#include "am_MainWindow.h"

#include "tools/BundleAdjustTool.h"
#include "tools/CanonicalTransformTool.h"
#include "tools/InitCamerasLandmarksTool.h"
#include "tools/NeckerReversalTool.h"
#include "tools/TrackFeaturesTool.h"
#include "tools/TrackFilterTool.h"
#include "tools/ConfigHelper.h"

#include "AboutDialog.h"
#include "MatchMatrixWindow.h"
#include "Project.h"
#include "vtkMaptkImageDataGeometryFilter.h"
#include "vtkMaptkImageUnprojectDepth.h"
#include "vtkMaptkCamera.h"

#include <maptk/version.h>
#include <maptk/local_geo_cs.h>

#include <vital/algo/video_input.h>
#include <vital/io/camera_io.h>
#include <vital/io/landmark_map_io.h>
#include <vital/io/metadata_io.h>
#include <vital/io/track_set_io.h>
#include <vital/types/metadata_map.h>
#include <arrows/core/match_matrix.h>

#include <vtksys/SystemTools.hxx>

#include <vtkImageData.h>
#include <vtkImageFlip.h>
#include <vtkImageImport.h>
#include <vtkImageReader2.h>
#include <vtkImageReader2Collection.h>
#include <vtkImageReader2Factory.h>
#include <vtkNew.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkXMLImageDataReader.h>

#include <qtEnumerate.h>
#include <qtIndexRange.h>
#include <qtMath.h>
#include <qtStlUtil.h>
#include <qtUiState.h>
#include <qtUiStateItem.h>

#include <QtGui/QApplication>
#include <QtGui/QColorDialog>
#include <QtGui/QDesktopServices>
#include <QtGui/QFileDialog>
#include <QtGui/QMessageBox>

#include <QtCore/QDebug>
#include <QtCore/QQueue>
#include <QtCore/QSignalMapper>
#include <QtCore/QTimer>
#include <QtCore/QUrl>

///////////////////////////////////////////////////////////////////////////////

//BEGIN miscellaneous helpers

namespace // anonymous
{

//-----------------------------------------------------------------------------
kwiver::vital::path_t kvPath(QString const& s)
{
  return stdString(s);
}


//-----------------------------------------------------------------------------
QString findUserManual()
{
  static auto const name = "telesculptor.html";
  static auto const product = "maptk";
  static auto const version = MAPTK_VERSION;

  auto const& prefix =
    QFileInfo(QApplication::applicationFilePath()).dir().absoluteFilePath("..");

  auto locations = QStringList();

  // Install location
  locations.append(QString("%1/share/doc/%2-%3").arg(prefix, product, version));

  // Build location
  locations.append(QString("%1/doc").arg(prefix));

  foreach (auto const& path, locations)
  {
    auto const fi = QFileInfo(QString("%1/user/%2").arg(path, name));
    if (fi.exists())
    {
      // Found manual
      return fi.canonicalFilePath();
    }
  }

  // Manual not found
  return QString();
}

//-----------------------------------------------------------------------------
QSet<QString> supportedImageExtensions()
{
  QSet<QString> result;

  auto const whitespace = QRegExp("\\s");

  // Get registered readers
  vtkNew<vtkImageReader2Collection> readers;
  vtkImageReader2Factory::GetRegisteredReaders(readers.GetPointer());

  // Extract extensions for each reader
  readers->InitTraversal();
  while (auto const reader = readers->GetNextItem())
  {
    auto const extensionList =
      QString::fromLocal8Bit(reader->GetFileExtensions());
    auto const& extensions =
      extensionList.split(whitespace, QString::SkipEmptyParts);

    foreach (auto const& ext, extensions)
    {
      result.insert(ext.mid(1).toLower());
    }
  }

  return result;
}

QSet<QString> supportedVideoExtensions()
{
  QSet<QString> result;

  // For now just add some common extensions we expect to encounter
  result.insert("mpeg");
  result.insert("mpg");
  result.insert("mp4");
  result.insert("avi");
  result.insert("wmw");
  result.insert("mov");

  return result;
}

//-----------------------------------------------------------------------------
QString makeFilters(QStringList extensions)
{
  auto result = QStringList();
  foreach (auto const& extension, extensions)
  {
    result.append("*." + extension);
  }
  return result.join(" ");
}

//-----------------------------------------------------------------------------
template <typename T>
class StateValue : public qtUiState::AbstractItem
{
public:
  StateValue(T const& defaultValue = T{}) : data{defaultValue} {}

  operator T() const { return this->data; }

  StateValue& operator=(T const& newValue)
  {
    this->data = newValue;
    return *this;
  }

  virtual QVariant value() const QTE_OVERRIDE
  {
    return QVariant::fromValue(this->data);
  }

  virtual void setValue(QVariant const& newValue) QTE_OVERRIDE
  {
    this->data = newValue.value<T>();
  }

protected:
  T data;
};

} // namespace <anonymous>

//END miscellaneous helpers

///////////////////////////////////////////////////////////////////////////////

//BEGIN MainWindowPrivate

//-----------------------------------------------------------------------------
class MainWindowPrivate
{
public:
  // Data structures
  struct FrameData
  {
    int id;
    vtkSmartPointer<vtkMaptkCamera> camera;

    QString depthMapPath; // Full path to depth map data
  };

  // Methods
  MainWindowPrivate()
    : activeTool(0)
    , toolUpdateActiveFrame(-1)
    , activeCameraIndex(-1) {}

  void addTool(AbstractTool* tool, MainWindow* mainWindow);

  void addCamera(kwiver::vital::camera_sptr const& camera);
  void addImage(QString const& imagePath);
  void addVideoSource(kwiver::vital::config_block_sptr const& config,
                      QString const& videoPath);

  void addFrame(kwiver::vital::camera_sptr const& camera, int id);

  kwiver::vital::camera_map_sptr cameraMap() const;
  void updateCameras(kwiver::vital::camera_map_sptr const&);
  bool updateCamera(kwiver::vital::frame_id_t frame,
                    kwiver::vital::camera_sptr cam);

  void setActiveCamera(int);
  void updateCameraView();

  vtkSmartPointer<vtkImageData> vitalToVtkImage(kwiver::vital::image& img);

  std::string getFrameName(kwiver::vital::frame_id_t frame);

  void loadImage(FrameData frame);
  void loadEmptyImage(vtkMaptkCamera* camera);

  void loadDepthMap(QString const& imagePath);

  void setActiveTool(AbstractTool* tool);

  // Member variables
  Ui::MainWindow UI;
  Am::MainWindow AM;
  qtUiState uiState;

  StateValue<QColor>* viewBackgroundColor;

  QTimer slideTimer;
  QSignalMapper toolDispatcher;

  QAction* toolSeparator;
  AbstractTool* activeTool;
  QList<AbstractTool*> tools;
  int toolUpdateActiveFrame;
  kwiver::vital::camera_map_sptr toolUpdateCameras;
  kwiver::vital::landmark_map_sptr toolUpdateLandmarks;
  kwiver::vital::feature_track_set_sptr toolUpdateTracks;

  QString videoPath;
  kwiver::vital::algo::video_input_sptr videoSource;
  kwiver::vital::timestamp currentVideoTimestamp;
  kwiver::vital::metadata_map::map_metadata_t videoMetadataMap;

  QList<FrameData> frames;
  kwiver::vital::feature_track_set_sptr tracks;
  kwiver::vital::landmark_map_sptr landmarks;

  kwiver::maptk::local_geo_cs localGeoCs;

  int activeCameraIndex;

  // Frames without a camera
  QQueue<int> orphanFrames;

  vtkNew<vtkXMLImageDataReader> depthReader;
  vtkNew<vtkMaptkImageUnprojectDepth> depthFilter;
  vtkNew<vtkMaptkImageDataGeometryFilter> depthGeometryFilter;

  // Current project
  std::shared_ptr<Project> currProject;
};

QTE_IMPLEMENT_D_FUNC(MainWindow)

//-----------------------------------------------------------------------------
void MainWindowPrivate::addTool(AbstractTool* tool, MainWindow* mainWindow)
{
  this->UI.menuCompute->insertAction(this->toolSeparator, tool);

  this->toolDispatcher.setMapping(tool, tool);

  QObject::connect(tool, SIGNAL(triggered()),
                   &this->toolDispatcher, SLOT(map()));
  QObject::connect(tool, SIGNAL(updated(std::shared_ptr<ToolData>)),
                   mainWindow, SLOT(acceptToolResults(std::shared_ptr<ToolData>)));
  QObject::connect(tool, SIGNAL(completed()),
                   mainWindow, SLOT(acceptToolFinalResults()));

  tool->setEnabled(false);

  this->tools.append(tool);
}

//-----------------------------------------------------------------------------
void MainWindowPrivate::addCamera(kwiver::vital::camera_sptr const& camera)
{
  if (this->orphanFrames.isEmpty())
  {
    this->addFrame(camera, this->frames.count() + 1);
    return;
  }

  auto& fd = this->frames[this->orphanFrames.dequeue()];

  fd.camera = vtkSmartPointer<vtkMaptkCamera>::New();
  fd.camera->SetCamera(camera);
  fd.camera->Update();

  this->UI.worldView->addCamera(fd.id, fd.camera);
  if (fd.id == this->activeCameraIndex)
  {
    this->UI.worldView->setActiveCamera(fd.id);
    this->updateCameraView();
  }
}

//-----------------------------------------------------------------------------
void MainWindowPrivate::addImage(QString const& imagePath)
{
  // TODO: Create/manage image list video source
}

//-----------------------------------------------------------------------------
void MainWindowPrivate::addVideoSource(kwiver::vital::config_block_sptr const& config,
                                       QString const& videoPath)
{
  // Save the configuration so independent video sources can be created for tools
  if (this->currProject)
  {
    this->currProject->projectConfig = config;
  }
  this->videoPath = videoPath;

  // Close the existing video source if it exists
  if(this->videoSource)
  {
    this->videoSource->close();
  }

  kwiver::vital::algo::video_input::set_nested_algo_configuration(
                                                            "video_reader",
                                                            config,
                                                            this->videoSource);

  try
  {
    if (this->videoSource)
    {
      this->videoSource->open(videoPath.toStdString());
    }

    // Get the video metadata
    if (this->videoSource)
    {
      videoMetadataMap = this->videoSource->metadata_map()->metadata();
    }

    // If we have metadata try and initialize cameras
    std::map<kwiver::vital::frame_id_t, kwiver::vital::camera_sptr> camMap;
    if (videoMetadataMap.size() > 0)
    {
      std::map<kwiver::vital::frame_id_t, kwiver::vital::metadata_sptr> mdMap;
      for (auto const& mdIter: this->videoMetadataMap)
      {
        // TODO: just using first element of metadata vector for now
        mdMap[mdIter.first] = mdIter.second[0];
      }

      auto baseCamera = kwiver::vital::simple_camera();
      camMap = kwiver::maptk::initialize_cameras_with_metadata(
          mdMap, baseCamera, this->localGeoCs);
    }

    // Add frames for video if needed
    auto numFrames = this->videoSource->num_frames();
    for (int i = this->frames.count(); i < numFrames; ++i)
    {
      // frames start at 1, list index starts at 0
      auto frameIdx = i + 1;
      if (camMap.find(frameIdx) != camMap.end())
      {
        this->addFrame(camMap[frameIdx], frameIdx);
      }
      else
      {
        this->orphanFrames.enqueue(i);
        this->addFrame(kwiver::vital::camera_sptr(), frameIdx);
      }
    }
  }
  catch (kwiver::vital::file_not_found_exception const& e)
  {
    qWarning() << e.what();
    this->videoSource->close();
    this->videoSource.reset();
  }
}

//-----------------------------------------------------------------------------
void MainWindowPrivate::addFrame(
  kwiver::vital::camera_sptr const& camera, int id)
{
  FrameData cd;

  cd.id = id;

  if (camera)
  {
    this->orphanFrames.clear();

    cd.camera = vtkSmartPointer<vtkMaptkCamera>::New();
    cd.camera->SetCamera(camera);
    cd.camera->Update();

    this->UI.worldView->addCamera(cd.id, cd.camera);
    this->UI.actionExportCameras->setEnabled(true);
  }

  this->frames.append(cd);

  this->UI.camera->setRange(1, this->frames.count());
  this->UI.cameraSpin->setRange(1, this->frames.count());

  // When the first camera is added, show it immediately and reset the camera
  // view, and enable slideshow controls
  if (this->frames.count() == 1)
  {
    this->UI.actionSlideshowPlay->setEnabled(true);
    this->UI.camera->setEnabled(true);
    this->UI.cameraSpin->setEnabled(true);

    this->setActiveCamera(1);
    this->UI.cameraView->resetView();
  }
}

//-----------------------------------------------------------------------------
kwiver::vital::camera_map_sptr MainWindowPrivate::cameraMap() const
{
  kwiver::vital::camera_map::map_camera_t map;

  foreach (auto i, qtIndexRange(this->frames.count()))
  {
    auto const& cd = this->frames[i];
    if (cd.camera)
    {
      map.insert(std::make_pair(static_cast<kwiver::vital::frame_id_t>(i),
                                cd.camera->GetCamera()));
    }
  }

  return std::make_shared<kwiver::vital::simple_camera_map>(map);
}

//-----------------------------------------------------------------------------
void MainWindowPrivate::updateCameras(
  kwiver::vital::camera_map_sptr const& cameras)
{
  auto const cameraCount = this->frames.count();
  auto allowExport = false;

  foreach (auto const& iter, cameras->cameras())
  {
    auto const index = static_cast<int>(iter.first);
    if (updateCamera(iter.first, iter.second))
    {
      allowExport = allowExport || iter.second;
    }
  }

  this->UI.actionExportCameras->setEnabled(allowExport);
}

//-----------------------------------------------------------------------------
bool MainWindowPrivate::updateCamera(kwiver::vital::frame_id_t frame,
                                     kwiver::vital::camera_sptr cam)
{
  if (frame > 0 && frame <= this->frames.count() && cam)
  {
    auto& cd = this->frames[frame - 1];
    if (!cd.camera)
    {
      cd.camera = vtkSmartPointer<vtkMaptkCamera>::New();
      this->UI.worldView->addCamera(cd.id, cd.camera);
    }
    cd.camera->SetCamera(cam);
    cd.camera->Update();

    if (cd.id == this->activeCameraIndex)
    {
      this->UI.worldView->setActiveCamera(cd.id);
      this->updateCameraView();
    }

    return true;
  }
  return false;
}

//-----------------------------------------------------------------------------
void MainWindowPrivate::setActiveCamera(int id)
{
  this->activeCameraIndex = id;
  this->UI.worldView->setActiveCamera(id);
  this->updateCameraView();

  auto& cd = this->frames[id-1];
  if (!cd.depthMapPath.isEmpty())
  {
    this->loadDepthMap(cd.depthMapPath);
  }

  // TODO: Uncomment once MeshColoration is working directly off video frames
  // UI.worldView->setVolumeCurrentFramePath(cd.imagePath);
}

//-----------------------------------------------------------------------------
void MainWindowPrivate::updateCameraView()
{
  if (this->activeCameraIndex < 1)
  {
    this->loadEmptyImage(0);
    this->UI.cameraView->setActiveFrame(static_cast<unsigned>(-1));
    this->UI.cameraView->clearLandmarks();
    return;
  }

  this->UI.cameraView->setActiveFrame(
    static_cast<unsigned>(this->activeCameraIndex));

  QHash<kwiver::vital::track_id_t, kwiver::vital::vector_2d> landmarkPoints;

  auto const& frame = this->frames[this->activeCameraIndex-1];

  // Show camera image
  this->loadImage(frame);

  if (!frame.camera)
  {
    // Can't show landmarks or residuals with no camera
    this->UI.cameraView->clearLandmarks();
    this->UI.cameraView->clearResiduals();
    return;
  }

  // Show landmarks
  this->UI.cameraView->clearLandmarks();
  if (this->landmarks)
  {
    // Map landmarks to camera space
    auto const& landmarks = this->landmarks->landmarks();
    foreach (auto const& lm, landmarks)
    {
      double pp[2];
      if (frame.camera->ProjectPoint(lm.second->loc(), pp))
      {
        // Add projected landmark to camera view
        auto const id = lm.first;
        this->UI.cameraView->addLandmark(id, pp[0], pp[1]);
        landmarkPoints.insert(id, kwiver::vital::vector_2d(pp[0], pp[1]));
      }
    }
  }

  // Show residuals
  this->UI.cameraView->clearResiduals();
  if (this->tracks)
  {
    auto const& tracks = this->tracks->tracks();
    foreach (auto const& track, tracks)
    {
      auto const& state = track->find(this->activeCameraIndex);
      if ( state == track->end() )
      {
        continue;
      }
      auto fts = std::dynamic_pointer_cast<kwiver::vital::feature_track_state>(*state);
      if ( fts && fts->feature)
      {
        auto const id = track->id();
        if (landmarkPoints.contains(id))
        {
          auto const& fp = fts->feature->loc();
          auto const& lp = landmarkPoints[id];
          this->UI.cameraView->addResidual(id, fp[0], fp[1], lp[0], lp[1]);
        }
      }
    }
  }
}

//-----------------------------------------------------------------------------
// TODO: move this method to a new implementation of image_container in a new
//       vtk arrow
vtkSmartPointer<vtkImageData>
MainWindowPrivate::vitalToVtkImage(kwiver::vital::image& img)
{
  auto imgTraits = img.pixel_traits();

  // Get the image type
  int imageType = VTK_VOID;
  switch (imgTraits.type)
  {
    case kwiver::vital::image_pixel_traits::UNSIGNED:
      imageType = VTK_UNSIGNED_CHAR;
      break;
    case kwiver::vital::image_pixel_traits::SIGNED:
      imageType = VTK_SIGNED_CHAR;
      break;
    case kwiver::vital::image_pixel_traits::FLOAT:
      imageType = VTK_FLOAT;
      break;
    default:
      imageType = VTK_VOID;
      break;
    // TODO: exception or error/warning message?
  }

  // convert to vtkFrameData
  vtkSmartPointer<vtkImageImport> imageImport =
    vtkSmartPointer<vtkImageImport>::New();
  imageImport->SetDataScalarType(imageType);
  imageImport->SetNumberOfScalarComponents(img.depth());
  imageImport->SetWholeExtent(0, img.width()-1, 0, img.height()-1, 0, 0);
  imageImport->SetDataExtentToWholeExtent();
  imageImport->SetImportVoidPointer(img.first_pixel());
  imageImport->Update();

  // Flip image so it has the correct axis for VTK
  vtkSmartPointer<vtkImageFlip> flipFilter =
    vtkSmartPointer<vtkImageFlip>::New();
  flipFilter->SetFilteredAxis(1); // flip x axis
  flipFilter->SetInputConnection(imageImport->GetOutputPort());
  flipFilter->Update();

  return flipFilter->GetOutput();
}

std::string MainWindowPrivate::getFrameName(kwiver::vital::frame_id_t frameId)
{
  if (videoMetadataMap.find(frameId) != videoMetadataMap.end())
  {
    auto mdVec = videoMetadataMap[frameId];
    for (auto const& md: mdVec)
    {
      if (md->has( kwiver::vital::VITAL_META_IMAGE_FILENAME ) ||
          md->has( kwiver::vital::VITAL_META_VIDEO_FILENAME ) )
      {
        return kwiver::vital::basename_from_metadata(md, frameId);
      }
    }
  }

  return kwiver::vital::basename_from_metadata(nullptr, frameId);
}

void MainWindowPrivate::loadEmptyImage(vtkMaptkCamera* camera)
{
  auto imageDimensions = QSize(1, 1);
  if (camera)
  {
    int w, h;
    camera->GetImageDimensions(w, h);
    imageDimensions = QSize(w, h);
  }

  this->UI.cameraView->setImageData(0, imageDimensions);
  this->UI.worldView->setImageData(0, imageDimensions);
}

//-----------------------------------------------------------------------------
void MainWindowPrivate::loadImage(FrameData frame)
{
  // TODO: check if seek vs next_frame is needed
  if (frame.id != this->currentVideoTimestamp.get_frame())
  {
    if (!this->videoSource ||
        !videoSource->seek_frame(this->currentVideoTimestamp, frame.id))
    {
      this->loadEmptyImage(frame.camera);
    }
  }

  // Get frame from video source
  if (this->videoSource)
  {
    // Advance video source if it hasn't been advanced
    if (!videoSource->good())
    {
      videoSource->next_frame(this->currentVideoTimestamp);
    }

    kwiver::vital::image frameImg;
    auto sourceImg = videoSource->frame_image()->get_image();

    // If image is interlaced it is already compatible with VTK
    if (sourceImg.d_step() == 1)
    {
      frameImg = sourceImg;
    }
    // Otherwise we need a deep copy to get it to be interlaced
    else
    {
      frameImg = kwiver::vital::image(sourceImg.width(),
                                      sourceImg.height(),
                                      sourceImg.depth(),
                                      true);
      frameImg.copy_from(sourceImg);
    }

    auto imageData = this->vitalToVtkImage(frameImg);
    int dimensions[3];
    imageData->GetDimensions(dimensions);

    // Test for errors
    if (dimensions[0] < 2 || dimensions[1] < 2)
    {
      qWarning() << "Failed to read image for frame " << frame.id;
      this->loadEmptyImage(frame.camera);
    }
    else
    {
      // If successful, update camera image dimensions
      if (frame.camera)
      {
        frame.camera->SetImageDimensions(dimensions);
      }

      // Set frame name in camera view
      std::string frameName = this->getFrameName(frame.id);
      this->UI.cameraView->setImagePath(QString::fromStdString(frameName));

      // Set image on views
      auto const size = QSize(dimensions[0], dimensions[1]);
      this->UI.cameraView->setImageData(imageData, size);
      this->UI.worldView->setImageData(imageData, size);
    }
  }
  else
  {
    this->loadEmptyImage(frame.camera);
  }
}

//-----------------------------------------------------------------------------
void MainWindowPrivate::loadDepthMap(QString const& imagePath)
{
  if (!vtksys::SystemTools::FileExists(qPrintable(imagePath), true))
  {
    qWarning() << "File doesn't exist: " << imagePath;
    return;
  }

  if (this->depthReader->GetFileName() &&
    !strcmp(this->depthReader->GetFileName(), qPrintable(imagePath)))
  {
    // No change to reader input... return without any update
    return;
  }

  this->depthReader->SetFileName(qPrintable(imagePath));

  this->UI.depthMapView->setValidDepthInput(true);
  this->UI.worldView->setValidDepthInput(true);

  this->depthFilter->SetCamera(this->frames[this->activeCameraIndex].camera);
  this->UI.worldView->updateDepthMap();
  this->UI.depthMapView->updateView(true);
  this->UI.actionExportDepthPoints->setEnabled(true);
}

//-----------------------------------------------------------------------------
void MainWindowPrivate::setActiveTool(AbstractTool* tool)
{
  // Disconnect cancel action
  QObject::disconnect(this->UI.actionCancelComputation, 0, this->activeTool, 0);

  // Update current tool
  this->activeTool = tool;

  // Connect cancel action
  if (tool)
  {
    QObject::connect(this->UI.actionCancelComputation, SIGNAL(triggered()),
                     tool, SLOT(cancel()));
    QObject::connect(this->UI.actionCancelComputation, SIGNAL(triggered()),
                     currProject.get(), SLOT(write()));
    QObject::connect(tool, SIGNAL(completed()),
                     currProject.get(), SLOT(write()));
  }

  auto const enableTools = !tool;
  auto const enableCancel = tool && tool->isCancelable();
  foreach (auto const& tool, this->tools)
  {
    tool->setEnabled(enableTools);
  }
  this->UI.actionCancelComputation->setEnabled(enableCancel);
  this->UI.actionOpen->setEnabled(enableTools);
}

//END MainWindowPrivate

///////////////////////////////////////////////////////////////////////////////

//BEGIN MainWindow

//-----------------------------------------------------------------------------
MainWindow::MainWindow(QWidget* parent, Qt::WindowFlags flags)
  : QMainWindow(parent, flags), d_ptr(new MainWindowPrivate)
{
  QTE_D();

  // Set up UI
  d->UI.setupUi(this);
  d->AM.setupActions(d->UI, this);

  d->toolSeparator =
    d->UI.menuCompute->insertSeparator(d->UI.actionCancelComputation);

  d->addTool(new TrackFeaturesTool(this), this);
  d->addTool(new InitCamerasLandmarksTool(this), this);
  d->addTool(new BundleAdjustTool(this), this);
  d->addTool(new CanonicalTransformTool(this), this);
  d->addTool(new NeckerReversalTool(this), this);
  d->addTool(new TrackFilterTool(this), this);

  d->UI.menuView->addSeparator();
  d->UI.menuView->addAction(d->UI.cameraViewDock->toggleViewAction());
  d->UI.menuView->addAction(d->UI.cameraSelectorDock->toggleViewAction());
  d->UI.menuView->addAction(d->UI.depthMapViewDock->toggleViewAction());

  d->UI.playSlideshowButton->setDefaultAction(d->UI.actionSlideshowPlay);
  d->UI.loopSlideshowButton->setDefaultAction(d->UI.actionSlideshowLoop);

  connect(d->UI.actionOpen, SIGNAL(triggered()), this, SLOT(openFile()));
  connect(d->UI.actionQuit, SIGNAL(triggered()), qApp, SLOT(quit()));

  connect(d->UI.actionNewProject, SIGNAL(triggered()), this, SLOT(newProject()));

  connect(d->UI.actionShowWorldAxes, SIGNAL(toggled(bool)),
          d->UI.worldView, SLOT(setAxesVisible(bool)));

  connect(d->UI.actionExportCameras, SIGNAL(triggered()),
          this, SLOT(saveCameras()));
  connect(d->UI.actionExportLandmarks, SIGNAL(triggered()),
          this, SLOT(saveLandmarks()));
  connect(d->UI.actionExportVolume, SIGNAL(triggered()),
          this, SLOT(saveVolume()));
  connect(d->UI.actionExportMesh, SIGNAL(triggered()),
          this, SLOT(saveMesh()));
  connect(d->UI.actionExportColoredMesh, SIGNAL(triggered()),
          this, SLOT(saveColoredMesh()));
  connect(d->UI.actionExportDepthPoints, SIGNAL(triggered()),
          this, SLOT(saveDepthPoints()));
  connect(d->UI.actionExportTracks, SIGNAL(triggered()),
          this, SLOT(saveTracks()));

  connect(d->UI.worldView, SIGNAL(depthMapEnabled(bool)),
          this, SLOT(enableSaveDepthPoints(bool)));

  connect(d->UI.actionShowMatchMatrix, SIGNAL(triggered()),
          this, SLOT(showMatchMatrix()));

  connect(&d->toolDispatcher, SIGNAL(mapped(QObject*)),
          this, SLOT(executeTool(QObject*)));

  connect(d->UI.actionSetBackgroundColor, SIGNAL(triggered()),
          this, SLOT(setViewBackroundColor()));

  connect(d->UI.actionAbout, SIGNAL(triggered()),
          this, SLOT(showAboutDialog()));
  connect(d->UI.actionShowManual, SIGNAL(triggered()),
          this, SLOT(showUserManual()));

  connect(&d->slideTimer, SIGNAL(timeout()), this, SLOT(nextSlide()));
  connect(d->UI.actionSlideshowPlay, SIGNAL(toggled(bool)),
          this, SLOT(setSlideshowPlaying(bool)));
  connect(d->UI.slideDelay, SIGNAL(valueChanged(int)),
          this, SLOT(setSlideDelay(int)));

  connect(d->UI.camera, SIGNAL(valueChanged(int)),
          this, SLOT(setActiveCamera(int)));

  connect(d->UI.worldView, SIGNAL(meshEnabled(bool)),
          this, SLOT(enableSaveMesh(bool)));

  connect(d->UI.worldView, SIGNAL(coloredMeshEnabled(bool)),
          this, SLOT(enableSaveColoredMesh(bool)));

  connect(d->UI.worldView, SIGNAL(depthMapThresholdsChanged()),
          d->UI.depthMapView, SLOT(updateThresholds()));

  connect(d->UI.depthMapViewDock, SIGNAL(visibilityChanged(bool)),
          d->UI.depthMapView, SLOT(updateView(bool)));

  this->setSlideDelay(d->UI.slideDelay->value());

#ifdef VTKWEBGLEXPORTER
  d->UI.actionWebGLScene->setVisible(true);
  connect(d->UI.actionWebGLScene, SIGNAL(triggered(bool)),
          this, SLOT(saveWebGLScene()));
#endif

  // Set up UI persistence and restore previous state
  auto const sdItem = new qtUiState::Item<int, QSlider>(
    d->UI.slideDelay, &QSlider::value, &QSlider::setValue);
  d->uiState.map("SlideDelay", sdItem);

  d->viewBackgroundColor = new StateValue<QColor>{Qt::black},
  d->uiState.map("ViewBackground", d->viewBackgroundColor);

  d->uiState.mapChecked("WorldView/Axes", d->UI.actionShowWorldAxes);

  d->uiState.mapState("Window/state", this);
  d->uiState.mapGeometry("Window/geometry", this);
  d->uiState.restore();

  d->UI.worldView->setBackgroundColor(*d->viewBackgroundColor);
  d->UI.cameraView->setBackgroundColor(*d->viewBackgroundColor);
  d->UI.depthMapView->setBackgroundColor(*d->viewBackgroundColor);

  // Hookup basic depth pipeline and pass geometry filter to relevant views
  d->depthFilter->SetInputConnection(d->depthReader->GetOutputPort());
  d->depthGeometryFilter->SetInputConnection(d->depthFilter->GetOutputPort());
  d->UI.worldView->setDepthGeometryFilter(d->depthGeometryFilter.GetPointer());
  d->UI.depthMapView->setDepthGeometryFilter(d->depthGeometryFilter.GetPointer());

  d->UI.worldView->resetView();
}

//-----------------------------------------------------------------------------
MainWindow::~MainWindow()
{
  QTE_D();
  d->uiState.save();
}

//-----------------------------------------------------------------------------
void MainWindow::openFile()
{
  static auto const imageFilters =
    makeFilters(supportedImageExtensions().toList());

  static auto const videoFilters =
    makeFilters(supportedVideoExtensions().toList());

  // TODO: Add image filters back once that is supported again.
  auto const paths = QFileDialog::getOpenFileNames(
    this, "Open File", QString(),
    "All Supported Files (*.conf *.txt *.ply *.krtd " + videoFilters + ");;"
    "Project configuration file (*.conf);;"
    "Video file (" + videoFilters + ");;"
    "Track file (*.txt);;"
    "Landmark file (*.ply);;"
    "Camera file (*.krtd);;"
    "All Files (*)");

  if (!paths.isEmpty())
  {
    this->openFiles(paths);
  }
}

//-----------------------------------------------------------------------------
void MainWindow::openFile(QString const& path)
{
  static auto const imageExtensions = supportedImageExtensions();
  static auto const videoExtensions = supportedVideoExtensions();

  auto const fi = QFileInfo(path);
  if (fi.suffix().toLower() == "conf")
  {
    this->loadProject(path);
  }
  else if (fi.suffix().toLower() == "txt")
  {
    this->loadTracks(path);
  }
  else if (fi.suffix().toLower() == "ply")
  {
    this->loadLandmarks(path);
  }
  else if (fi.suffix().toLower() == "krtd")
  {
    this->loadCamera(path);
  }
  else if (imageExtensions.contains(fi.suffix().toLower()))
  {
    this->loadImage(path);
  }
  else if (videoExtensions.contains(fi.suffix().toLower()))
  {
    this->loadVideo(path);
  }
  else
  {
    qWarning() << "Don't know how to read file" << path
               << "(unrecognized extension)";
  }
}

//-----------------------------------------------------------------------------
void MainWindow::openFiles(QStringList const& paths)
{
  foreach (auto const& path, paths)
  {
    this->openFile(path);
  }
}

//-----------------------------------------------------------------------------
void MainWindow::newProject()
{
  QTE_D();

  auto const dirname = QFileDialog::getExistingDirectory(
    this, "Select Project Directory");

  if (!dirname.isEmpty())
  {
    // Set the current working directory to the project directory
    if (!QDir::setCurrent(dirname))
    {
      qWarning() << "Unable to set current working directory to "
                 << "project directory: " << dirname;
    }

    d->currProject.reset();
    d->currProject = std::shared_ptr<Project>(new Project(dirname));

    if (d->videoSource)
    {
      d->currProject->videoPath = d->videoPath;
      auto config = ConfigHelper::readConfig("gui_video_reader.conf");
      d->currProject->projectConfig->merge_config(config);
    }

    saveCameras(d->currProject->cameraPath);
    d->currProject->projectConfig->set_value("output_krtd_dir", kvPath(
      d->currProject->getContingentRelativePath(d->currProject->cameraPath)));

    if (!d->localGeoCs.origin().is_empty())
    {
      saveGeoOrigin(d->currProject->geoOriginFile);
    }

    d->currProject->write();
  }

  foreach (auto const& tool, d->tools)
  {
    tool->setEnabled(true);
  }
}

//-----------------------------------------------------------------------------
void MainWindow::loadProject(QString const& path)
{
  QTE_D();

  d->currProject = std::make_shared<Project>();
  if (!d->currProject->read(path))
  {
    qWarning() << "Failed to load project from" << path; // TODO dialog?
    d->currProject.reset();
    return;
  }

  // Set the current working directory to the project directory
  if (!QDir::setCurrent(d->currProject->workingDir.absolutePath()))
  {
    qWarning() << "Unable to set current working directory to "
      << "project directory: " << d->currProject->workingDir.absolutePath();
  }

  // Get the video source
  if (d->currProject->projectConfig->has_value("video_reader:type"))
  {
    d->addVideoSource(d->currProject->projectConfig, d->currProject->videoPath);
  }

  // Load tracks
  if (d->currProject->projectConfig->has_value("input_track_file") ||
      d->currProject->projectConfig->has_value("output_tracks_file"))
  {
    this->loadTracks(d->currProject->tracksPath);
  }

  // Load landmarks
  if (d->currProject->projectConfig->has_value("output_ply_file"))
  {
    this->loadLandmarks(d->currProject->landmarksPath);
  }

  // Load cameras
  if (d->currProject->projectConfig->has_value("output_krtd_dir"))
  {
    foreach (auto const& frame, d->frames)
    {
      auto frameName = QString::fromStdString(d->getFrameName(frame.id) + ".krtd");

      try
      {
        auto const& camera = kwiver::vital::read_krtd_file(
          kvPath(frameName), kvPath(d->currProject->cameraPath));

        // Add camera to scene
        d->updateCamera(frame.id, camera);
      }
      catch (...)
      {
        qWarning() << "failed to read camera file " << frameName
                   << " from " << d->currProject->cameraPath;
      }
    }
  }

  // Associate depth maps with cameras
  foreach (auto dm, qtEnumerate(d->currProject->depthMaps))
  {
    auto const i = dm.key();
    if (i >= 0 && i < d->frames.count())
    {
      d->frames[i].depthMapPath = dm.value();
    }

    if (i == d->activeCameraIndex)
    {
      d->loadDepthMap(dm.value());
    }
  }

#ifdef VTKWEBGLEXPORTER
  d->UI.actionWebGLScene->setEnabled(true);
#endif

  // Load volume
  if (d->currProject->projectConfig->has_value("volume_file"))
  {
    d->UI.worldView->loadVolume(d->currProject->volumePath, d->frames.size(),
                                d->currProject->cameraPath, d->currProject->videoPath);
  }

  if (d->currProject->projectConfig->has_value("geo_origin_file"))
  {
    if (vtksys::SystemTools::FileExists(
        d->currProject->geoOriginFile.toStdString(), true))
    {
      kwiver::maptk::read_local_geo_cs_from_file(
        d->localGeoCs, d->currProject->geoOriginFile.toStdString());
    }
    else
    {
      qWarning() << "Failed to open geo origin file "
        << d->currProject->geoOriginFile << ". File does not exist.";
    }
  }

  d->UI.worldView->queueResetView();

  foreach (auto const& tool, d->tools)
  {
    tool->setEnabled(true);
  }
}

//-----------------------------------------------------------------------------
void MainWindow::loadImage(QString const& path)
{
  QTE_D();
  d->addImage(path);
}

void MainWindow::loadVideo(QString const& path)
{
  QTE_D();

  auto config = ConfigHelper::readConfig("gui_video_reader.conf");
  if (d->currProject)
  {
    d->currProject->projectConfig->merge_config(config);
    d->currProject->videoPath = path;
  }

  try
  {
    d->addVideoSource(config, path);
  }
  catch (std::exception const& e)
  {
    QMessageBox::critical(
      this, "Error loading video\n",
      e.what());
  }

  if (d->currProject)
  {
    saveCameras(d->currProject->cameraPath);
    d->currProject->projectConfig->set_value("output_krtd_dir", kvPath(
      d->currProject->getContingentRelativePath(d->currProject->cameraPath)));

    if (!d->localGeoCs.origin().is_empty())
    {
      saveGeoOrigin(d->currProject->geoOriginFile);
    }

    d->currProject->write();
  }

  d->UI.worldView->queueResetView();
}

//-----------------------------------------------------------------------------
void MainWindow::loadCamera(QString const& path)
{
  QTE_D();

  try
  {
    auto const& camera = kwiver::vital::read_krtd_file(kvPath(path));
    d->addCamera(camera);
  }
  catch (...)
  {
    qWarning() << "failed to read camera from" << path;
  }
}

//-----------------------------------------------------------------------------
void MainWindow::loadTracks(QString const& path)
{
  QTE_D();

  try
  {
    using namespace kwiver::vital;
    auto tracks = read_feature_track_file(kvPath(path));
    if (tracks)
    {
      // check for older zero-based track files
      if (tracks->first_frame() == 0)
      {
        qWarning() << "Loaded tracks have zero-based indexing, "
                      "shifting to one-based indexing";
        // shift tracks to start with frame one
        std::vector<track_sptr> new_tracks;
        for (auto track : tracks->tracks())
        {
          auto new_track = track::create(track->data());
          new_track->set_id(track->id());
          for (auto ts : *track)
          {
            auto fts = std::dynamic_pointer_cast<feature_track_state>(ts);
            auto new_fts = std::make_shared<feature_track_state>(ts->frame()+1,
                                                fts->feature, fts->descriptor);
            new_track->append(new_fts);
          }
          new_tracks.push_back(new_track);
        }
        tracks = std::make_shared<feature_track_set>(new_tracks);
      }

      d->tracks = tracks;
      d->updateCameraView();

      for (auto const& track : tracks->tracks())
      {
        d->UI.cameraView->addFeatureTrack(*track);
      }

      d->UI.actionExportTracks->setEnabled(
          d->tracks && d->tracks->size());

      d->UI.actionShowMatchMatrix->setEnabled(!tracks->tracks().empty());
    }
  }
  catch (...)
  {
    qWarning() << "failed to read tracks from" << path;
  }
}

//-----------------------------------------------------------------------------
void MainWindow::loadLandmarks(QString const& path)
{
  QTE_D();

  try
  {
    auto const& landmarks = kwiver::vital::read_ply_file(kvPath(path));
    if (landmarks)
    {
      d->landmarks = landmarks;
      d->UI.worldView->setLandmarks(*landmarks);
      d->UI.cameraView->setLandmarksData(*landmarks);

      d->UI.actionExportLandmarks->setEnabled(
        d->landmarks && d->landmarks->size());

      d->updateCameraView();
    }
  }
  catch (...)
  {
    qWarning() << "failed to read landmarks from" << path;
  }
}

//-----------------------------------------------------------------------------
void MainWindow::saveLandmarks()
{
  auto const path = QFileDialog::getSaveFileName(
    this, "Export Landmarks", QString(),
    "Landmark file (*.ply);;"
    "All Files (*)");

  if (!path.isEmpty())
  {
    this->saveLandmarks(path, false);
  }
}

//-----------------------------------------------------------------------------
void MainWindow::saveLandmarks(QString const& path, bool writeToProject)
{
  QTE_D();

  try
  {
    kwiver::vital::write_ply_file(d->landmarks, kvPath(path));

    if (writeToProject && d->currProject)
    {
      d->currProject->projectConfig->set_value("output_ply_file", kvPath(
        d->currProject->getContingentRelativePath(path)));
    }
  }
  catch (...)
  {
    auto const msg =
      QString("An error occurred while exporting landmarks to \"%1\". "
              "The output file may not have been written correctly.");
    QMessageBox::critical(this, "Export error", msg.arg(d->currProject->landmarksPath));
  }
}

//-----------------------------------------------------------------------------
void MainWindow::saveTracks()
{
  auto const path = QFileDialog::getSaveFileName(
    this, "Export Tracks", QString(),
    "Track file (*.txt);;"
    "All Files (*)");

  if (!path.isEmpty())
  {
    this->saveTracks(path, false);
  }
}

//-----------------------------------------------------------------------------
void MainWindow::saveTracks(QString const& path, bool writeToProject)
{
  QTE_D();

  try
  {
    kwiver::vital::write_feature_track_file(d->tracks, kvPath(path));

    if (writeToProject && d->currProject)
    {
      d->currProject->projectConfig->set_value("output_tracks_file", kvPath(
        d->currProject->getContingentRelativePath(path)));
    }
  }
  catch (...)
  {
    auto const msg =
      QString("An error occurred while exporting tracks to \"%1\". "
              "The output file may not have been written correctly.");
    QMessageBox::critical(this, "Export error", msg.arg(path));
  }
}

//-----------------------------------------------------------------------------
void MainWindow::saveCameras()
{
  auto const path = QFileDialog::getExistingDirectory(this, "Export Cameras");

  if (!path.isEmpty())
  {
    this->saveCameras(path, false);
  }
}

//-----------------------------------------------------------------------------
void MainWindow::saveCameras(QString const& path, bool writeToProject)
{
  QTE_D();

  auto out = QHash<QString, kwiver::vital::camera_sptr>();
  auto willOverwrite = QStringList();

  foreach (auto i, qtIndexRange(d->frames.count()))
  {
    auto const& cd = d->frames[i];
    if (cd.camera)
    {
      auto const camera = cd.camera->GetCamera();
      if (camera)
      {
        auto cameraName = QString::fromStdString(d->getFrameName(cd.id) + ".krtd");
        auto const filepath = d->currProject->cameraPath + "/" + cameraName;
        out.insert(filepath, camera);

        if (QFileInfo(filepath).exists())
        {
          willOverwrite.append(filepath);
        }
      }
    }
  }

  // warn about overwriting files only if not auto-saving to the project
  if (!writeToProject && !willOverwrite.isEmpty())
  {
    QMessageBox mb(QMessageBox::Warning, "Confirm overwrite",
                   "One or more files will be overwritten by this operation. "
                   "Do you wish to continue?", QMessageBox::Cancel, this);

    mb.addButton("&Overwrite", QMessageBox::AcceptRole);
    mb.setDetailedText("The following file(s) will be overwritten:\n  " +
                       willOverwrite.join("  \n"));

    if (mb.exec() != QDialog::Accepted)
    {
      // User canceled operation
      return;
    }
  }

  auto errors = QStringList();
  foreach (auto const& iter, qtEnumerate(out))
  {
    try
    {
      kwiver::vital::write_krtd_file(*iter.value(), kvPath(iter.key()));
    }
    catch (...)
    {
      errors.append(iter.key());
    }
  }

  if (writeToProject && d->currProject)
  {
    d->currProject->projectConfig->set_value("output_krtd_dir", kvPath(
      d->currProject->getContingentRelativePath(path)));
  }

  if (!errors.isEmpty())
  {
    auto const msg =
      QString("Error(s) occurred while exporting cameras to \"%1\". "
              "One or more output files may not have been written correctly.");

    QMessageBox mb(QMessageBox::Critical, "Export error",
                   msg.arg(d->currProject->cameraPath), QMessageBox::Ok, this);

    mb.setDetailedText("Error writing the following file(s):\n  " +
                       errors.join("  \n"));

    mb.exec();
  }
}

//-----------------------------------------------------------------------------
void MainWindow::enableSaveDepthPoints(bool state)
{
  QTE_D();

  if (state && d->depthGeometryFilter->GetOutput()->GetNumberOfVerts() <= 0)
  {
    state = false;
  }
  d->UI.actionExportDepthPoints->setEnabled(state);
}

//-----------------------------------------------------------------------------
void MainWindow::saveDepthPoints()
{
  auto const path = QFileDialog::getSaveFileName(
    this, "Export Depth Point Cloud", QString(),
    "PLY file (*.ply);;"
    "All Files (*)");

  if (!path.isEmpty())
  {
    this->saveDepthPoints(path);
  }
}

//-----------------------------------------------------------------------------
void MainWindow::saveDepthPoints(QString const& path)
{
  QTE_D();

  try
  {
    d->UI.worldView->saveDepthPoints(path);
    d->currProject->projectConfig->set_value("depthmaps_images_file",
      kvPath(d->currProject->getContingentRelativePath(path)));
    d->currProject->write();
  }
  catch (...)
  {
    auto const msg =
      QString("An error occurred while exporting depth points to \"%1\". "
              "The output file may not have been written correctly.");
    QMessageBox::critical(this, "Export error", msg.arg(path));
  }
}

void MainWindow::saveGeoOrigin(QString const& path)
{
  QTE_D();

  d->currProject->projectConfig->set_value("geo_origin_file", kvPath(
    d->currProject->getContingentRelativePath(path)));
  kwiver::maptk::write_local_geo_cs_to_file(d->localGeoCs, path.toStdString());
}

//-----------------------------------------------------------------------------
void MainWindow::saveWebGLScene()
{
#ifdef VTKWEBGLEXPORTER
  QTE_D();

  auto const path = QFileDialog::getSaveFileName(
    this, "Export Scene to WebGL", QString(),
    "WebGL scene file (*.html);;"
    "All Files (*)");

  if (!path.isEmpty())
  {
    d->UI.worldView->exportWebGLScene(path);
  }
#endif
}

//-----------------------------------------------------------------------------
void MainWindow::enableSaveMesh(bool state)
{
  QTE_D();

  d->UI.actionExportVolume->setEnabled(state);
  d->UI.actionExportMesh->setEnabled(state);
}

//-----------------------------------------------------------------------------
void MainWindow::enableSaveColoredMesh(bool state)
{
  QTE_D();

  d->UI.actionExportColoredMesh->setEnabled(state);
}

//-----------------------------------------------------------------------------
void MainWindow::saveMesh()
{
  QTE_D();

  auto const path = QFileDialog::getSaveFileName(
    this, "Export Mesh", QString("mesh.vtp"),
    "Mesh file (*.vtp);;"
    "All Files (*)");

  if (!path.isEmpty())
  {
    d->UI.worldView->saveMesh(path);
  }
}

//-----------------------------------------------------------------------------
void MainWindow::saveVolume()
{
  QTE_D();

  auto const path = QFileDialog::getSaveFileName(
    this, "Export Volume", QString("volume.vts"),
    "Mesh file (*.vts);;"
    "All Files (*)");

  if (!path.isEmpty())
  {
    d->UI.worldView->saveVolume(path);
    d->currProject->volumePath = d->currProject->getContingentRelativePath(path);
    d->currProject->projectConfig->set_value("volume_file",
      kvPath(d->currProject->volumePath));
    d->currProject->write();
  }
}

//-----------------------------------------------------------------------------
void MainWindow::saveColoredMesh()
{
  QTE_D();

  auto const path = QFileDialog::getSaveFileName(
    this, "Export Colored Mesh", QString("colored_mesh.vtp"),
    "VTK Polydata (*.vtp);;"
    "PLY File (*.ply);;"
    "All Files (*)");

  if (!path.isEmpty())
  {
    d->UI.worldView->saveColoredMesh(path);
  }
}

//-----------------------------------------------------------------------------
void MainWindow::setSlideDelay(int delayExp)
{
  QTE_D();

  static auto const ttFormat =
    QString("%1 (%2)").arg(d->UI.slideDelay->toolTip());

  auto const de = static_cast<double>(delayExp) * 0.1;
  auto const delay = qRound(qPow(10.0, de));
  d->slideTimer.setInterval(delay);

  if (delay < 1000)
  {
    auto const fps = 1e3 / delay;
    auto const dt = QString("%1 / sec").arg(fps, 0, 'f', 1);
    d->UI.slideDelay->setToolTip(ttFormat.arg(dt));
  }
  else
  {
    auto const dt = QString("%1 sec").arg(delay / 1e3, 0, 'f', 1);
    d->UI.slideDelay->setToolTip(ttFormat.arg(dt));
  }
}

//-----------------------------------------------------------------------------
void MainWindow::setSlideshowPlaying(bool playing)
{
  QTE_D();
  if (playing)
  {
    if (d->UI.camera->value() == d->UI.camera->maximum())
    {
      d->UI.camera->triggerAction(QAbstractSlider::SliderToMinimum);
    }
    d->slideTimer.start();
  }
  else
  {
    d->slideTimer.stop();
  }

  d->UI.camera->setEnabled(!playing);
}

//-----------------------------------------------------------------------------
void MainWindow::nextSlide()
{
  QTE_D();

  if (d->UI.camera->value() == d->UI.camera->maximum())
  {
    if (d->UI.actionSlideshowLoop->isChecked())
    {
      d->UI.camera->triggerAction(QAbstractSlider::SliderToMinimum);
    }
    else
    {
      d->UI.actionSlideshowPlay->setChecked(false);
    }
  }
  else
  {
    d->UI.camera->triggerAction(QAbstractSlider::SliderSingleStepAdd);
  }
}

//-----------------------------------------------------------------------------
void MainWindow::setActiveCamera(int id)
{
  QTE_D();

  if (id < 1 || id > d->frames.count())
  {
    qDebug() << "MainWindow::setActiveCamera:"
             << " requested ID" << id << "is invalid";
    return;
  }

  d->setActiveCamera(id);
}

//-----------------------------------------------------------------------------
void MainWindow::executeTool(QObject* object)
{
  QTE_D();

  try
  {
    auto const tool = qobject_cast<AbstractTool*>(object);
    if (tool && !d->activeTool)
    {
      d->setActiveTool(tool);
      tool->setActiveFrame(d->activeCameraIndex);
      tool->setTracks(d->tracks);
      tool->setCameras(d->cameraMap());
      tool->setLandmarks(d->landmarks);
      tool->setVideoPath(d->videoPath.toStdString());
      tool->setConfig(d->currProject->projectConfig);

      if (!tool->execute())
      {
        d->setActiveTool(0);
      }
    }
  }
  catch (std::exception const& e)
  {
    QString message("The tool failed with the following error:\n");
    message += e.what();
    QMessageBox::critical(
      this, "Error in Tool",
      message);
  }
}

//-----------------------------------------------------------------------------
void MainWindow::acceptToolFinalResults()
{
  QTE_D();
  if (d->activeTool)
  {
    acceptToolResults(d->activeTool->data());
    saveToolResults();
  }
  d->setActiveTool(0);
}

//-----------------------------------------------------------------------------
void MainWindow::acceptToolResults(std::shared_ptr<ToolData> data)
{
  QTE_D();
  // if all the update variables are Null then trigger a GUI update after
  // extracting the data otherwise we've already triggered an update that
  // hasn't happened yet, so don't trigger another
  bool updateNeeded = !d->toolUpdateCameras &&
                      !d->toolUpdateLandmarks &&
                      !d->toolUpdateTracks &&
                      d->toolUpdateActiveFrame < 0;

  if (d->activeTool)
  {
    auto const outputs = d->activeTool->outputs();

    d->toolUpdateCameras = NULL;
    d->toolUpdateLandmarks = NULL;
    d->toolUpdateTracks = NULL;
    d->toolUpdateActiveFrame = -1;
    if (outputs.testFlag(AbstractTool::Cameras))
    {
      d->toolUpdateCameras = data->cameras;
    }
    if (outputs.testFlag(AbstractTool::Landmarks))
    {
      d->toolUpdateLandmarks = data->landmarks;
    }
    if (outputs.testFlag(AbstractTool::Tracks))
    {
      d->toolUpdateTracks = data->tracks;
    }
    if (outputs.testFlag(AbstractTool::ActiveFrame))
    {
      d->toolUpdateActiveFrame = static_cast<int>(data->activeFrame);
    }
  }

  if(updateNeeded)
  {
    QTimer::singleShot(1000, this, SLOT(updateToolResults()));
  }
}


//-----------------------------------------------------------------------------
void MainWindow::saveToolResults()
{
  QTE_D();

  if (d->activeTool)
  {
    auto const outputs = d->activeTool->outputs();

    if (outputs.testFlag(AbstractTool::Cameras))
    {
      saveCameras(d->currProject->cameraPath);
    }
    if (outputs.testFlag(AbstractTool::Landmarks))
    {
      saveLandmarks(d->currProject->landmarksPath);
    }
    if (outputs.testFlag(AbstractTool::Tracks))
    {
      saveTracks(d->currProject->tracksPath);
    }

    if (!d->localGeoCs.origin().is_empty())
    {
      saveGeoOrigin(d->currProject->geoOriginFile);
    }

    d->currProject->write();
  }
}

//-----------------------------------------------------------------------------
void MainWindow::updateToolResults()
{
  QTE_D();

  if (d->toolUpdateCameras)
  {
    d->updateCameras(d->toolUpdateCameras);
    d->toolUpdateCameras = NULL;
  }
  if (d->toolUpdateLandmarks)
  {
    d->landmarks = d->toolUpdateLandmarks;
    d->UI.worldView->setLandmarks(*d->landmarks);

    d->UI.actionExportLandmarks->setEnabled(
      d->landmarks && d->landmarks->size());
    d->toolUpdateLandmarks = NULL;
  }
  if (d->toolUpdateTracks)
  {
    d->tracks = d->toolUpdateTracks;
    d->UI.cameraView->clearFeatureTracks();
    d->updateCameraView();

    foreach (auto const& track, d->tracks->tracks())
    {
      d->UI.cameraView->addFeatureTrack(*track);
    }
    d->UI.actionExportTracks->setEnabled(
        d->tracks && d->tracks->size());

    d->UI.actionShowMatchMatrix->setEnabled(!d->tracks->tracks().empty());
    d->toolUpdateTracks = NULL;
  }
  if (d->toolUpdateActiveFrame >= 0)
  {
    d->UI.camera->setValue(d->toolUpdateActiveFrame);
    this->setActiveCamera(d->toolUpdateActiveFrame);
    d->toolUpdateActiveFrame = -1;
  }

  if (!d->frames.isEmpty())
  {
    d->setActiveCamera(d->activeCameraIndex);
  }
}

//-----------------------------------------------------------------------------
void MainWindow::showMatchMatrix()
{
  QTE_D();

  if (d->tracks)
  {
    // Get matrix
    auto frames = std::vector<kwiver::vital::frame_id_t>();
    auto const mm = kwiver::arrows::match_matrix(d->tracks, frames);

    // Show window
    auto window = new MatchMatrixWindow();
    window->setMatrix(mm, frames);
    window->show();
  }
}

//-----------------------------------------------------------------------------
void MainWindow::setViewBackroundColor()
{
  QTE_D();

  QColorDialog dlg;
  dlg.setCurrentColor(*d->viewBackgroundColor);
  if (dlg.exec() == QDialog::Accepted)
  {
    *d->viewBackgroundColor = dlg.currentColor();
    d->UI.worldView->setBackgroundColor(*d->viewBackgroundColor);
    d->UI.cameraView->setBackgroundColor(*d->viewBackgroundColor);
    d->UI.depthMapView->setBackgroundColor(*d->viewBackgroundColor);
  }
}

//-----------------------------------------------------------------------------
void MainWindow::showAboutDialog()
{
  AboutDialog dlg(this);
  dlg.exec();
}

//-----------------------------------------------------------------------------
void MainWindow::showUserManual()
{
  auto const path = findUserManual();
  if (!path.isEmpty())
  {
    auto const& uri = QUrl::fromLocalFile(path);
    QDesktopServices::openUrl(uri);
  }
  else
  {
    QMessageBox::information(
      this, "Not found",
      "The user manual could not be located. Please check your installation.");
  }
}

//END MainWindow
