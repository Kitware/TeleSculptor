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
#include "GuiCommon.h"

#include "ui_MainWindow.h"
#include "am_MainWindow.h"

#include "tools/BundleAdjustTool.h"
#include "tools/CanonicalTransformTool.h"
#include "tools/ComputeDepthTool.h"
#include "tools/InitCamerasLandmarksTool.h"
#include "tools/NeckerReversalTool.h"
#include "tools/SaveFrameTool.h"
#include "tools/SaveKeyFrameTool.h"
#include "tools/TrackFeaturesSprokitTool.h"
#include "tools/TrackFeaturesTool.h"
#include "tools/TrackFilterTool.h"
#include "tools/TriangulateTool.h"

#include "AboutDialog.h"
#include "MatchMatrixWindow.h"
#include "Project.h"
#include "VideoImport.h"
#include "vtkMaptkImageDataGeometryFilter.h"
#include "vtkMaptkImageUnprojectDepth.h"
#include "vtkMaptkCamera.h"

#include <maptk/version.h>
#include <maptk/local_geo_cs.h>

#include <vital/algo/video_input.h>
#include <vital/io/camera_io.h>
#include <vital/io/landmark_map_io.h>
#include <vital/io/track_set_io.h>
#include <vital/types/camera_perspective.h>
#include <vital/types/metadata_map.h>
#include <arrows/core/match_matrix.h>
#include <arrows/core/track_set_impl.h>

#include <vtkBox.h>
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
#include <vtkXMLImageDataWriter.h>

#include <qtEnumerate.h>
#include <qtIndexRange.h>
#include <qtMath.h>
#include <qtStlUtil.h>
#include <qtUiState.h>
#include <qtUiStateItem.h>

#include <QApplication>
#include <QColorDialog>
#include <QDebug>
#include <QDesktopServices>
#include <QFileDialog>
#include <QMessageBox>
#include <QPushButton>
#include <QQueue>
#include <QSignalMapper>
#include <QTimer>
#include <QUrl>

namespace kv = kwiver::vital;

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

//-----------------------------------------------------------------------------
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
  result.insert("txt"); // image list

  return result;
}

//-----------------------------------------------------------------------------
std::string imageConfigForPath(QString const& path, QString const& type)
{
  static const auto configTemplate = QString{"gui_%1_%2_reader.conf"};

  const auto ext = QFileInfo{path}.suffix();
  const QString mode = (ext.toLower() == "txt" ? "list" : "video");
  return stdString(configTemplate.arg(type, mode));
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
  MainWindowPrivate(MainWindow* mainWindow);

  void addTool(AbstractTool* tool, MainWindow* mainWindow);

  void addCamera(kwiver::vital::camera_perspective_sptr const& camera);
  void addImage(QString const& imagePath);
  void addVideoSource(kwiver::vital::config_block_sptr const& config,
                      QString const& videoPath);
  void addMaskSource(kwiver::vital::config_block_sptr const& config,
                     QString const& maskPath);

  void addFrame(kwiver::vital::camera_perspective_sptr const& camera, int id);
  void updateFrames(std::shared_ptr<kwiver::vital::metadata_map::map_metadata_t>);

  kwiver::vital::camera_map_sptr cameraMap() const;
  void updateCameras(kwiver::vital::camera_map_sptr const&);
  bool updateCamera(kwiver::vital::frame_id_t frame,
                    kwiver::vital::camera_perspective_sptr cam);

  void setActiveCamera(int);
  void updateCameraView();

  vtkSmartPointer<vtkImageData> vitalToVtkImage(kwiver::vital::image& img);

  std::string getFrameName(kwiver::vital::frame_id_t frame);

  void loadImage(FrameData frame);
  void loadEmptyImage(vtkMaptkCamera* camera);

  void loadDepthMap(QString const& imagePath);

  void setActiveTool(AbstractTool* tool);
  void updateProgress(QObject* object,
                      const QString& description = QString(""),
                      int value = 0);

  // Member variables
  Ui::MainWindow UI;
  Am::MainWindow AM;
  qtUiState uiState;

  StateValue<QColor>* viewBackgroundColor;

  QTimer slideTimer;
  QSignalMapper toolDispatcher;

  QAction* toolSeparator;
  QMenu* toolMenu;
  AbstractTool* activeTool;
  QList<AbstractTool*> tools;
  int toolUpdateActiveFrame;
  kwiver::vital::camera_map_sptr toolUpdateCameras;
  kwiver::vital::landmark_map_sptr toolUpdateLandmarks;
  kwiver::vital::feature_track_set_sptr toolUpdateTracks;
  vtkSmartPointer<vtkImageData> toolUpdateDepth;

  kv::config_block_sptr freestandingConfig = kv::config_block::empty_config();

  QString videoPath;
  QString maskPath;
  kwiver::vital::algo::video_input_sptr videoSource;
  kwiver::vital::algo::video_input_sptr maskSource;
  kwiver::vital::timestamp currentVideoTimestamp;
  kwiver::vital::metadata_map::map_metadata_t videoMetadataMap;
  kwiver::vital::frame_id_t advanceInterval;

  QMap<kwiver::vital::frame_id_t, FrameData> frames;
  kwiver::vital::feature_track_set_sptr tracks;
  kwiver::vital::landmark_map_sptr landmarks;
  vtkSmartPointer<vtkImageData> activeDepth;
  int activeDepthFrame;

  kwiver::maptk::local_geo_cs localGeoCs;

  int activeCameraIndex;

  VideoImport videoImporter;

  // Frames without a camera
  QQueue<int> orphanFrames;

  vtkNew<vtkXMLImageDataReader> depthReader;
  vtkNew<vtkMaptkImageUnprojectDepth> depthFilter;
  vtkNew<vtkMaptkImageDataGeometryFilter> depthGeometryFilter;

  vtkNew<vtkBox> roi;

  // Current project
  QScopedPointer<Project> project;

  // Progress tracking
  QHash<QObject*, int> progressIds;
};

QTE_IMPLEMENT_D_FUNC(MainWindow)

//-----------------------------------------------------------------------------
MainWindowPrivate::MainWindowPrivate(MainWindow* mainWindow)
    : activeTool(0)
    , toolUpdateActiveFrame(-1)
    , advanceInterval(1)
    , activeCameraIndex(-1)
{
  QObject::connect(&videoImporter, SIGNAL(updated(int)),
                   mainWindow, SLOT(addFrame(int)));
  QObject::connect(&videoImporter, SIGNAL(updateProgress(QString, int)),
                   mainWindow, SLOT(updateVideoImportProgress(QString, int)));
  QObject::connect(
    &videoImporter,
    SIGNAL(completed(std::shared_ptr<kwiver::vital::metadata_map::map_metadata_t>)),
    mainWindow,
    SLOT(updateFrames(std::shared_ptr<kwiver::vital::metadata_map::map_metadata_t>)));
}

//-----------------------------------------------------------------------------
void MainWindowPrivate::addTool(AbstractTool* tool, MainWindow* mainWindow)
{
  this->toolMenu->insertAction(this->toolSeparator, tool);

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
void MainWindowPrivate::addCamera(kwiver::vital::camera_perspective_sptr const& camera)
{
  if (!this->orphanFrames.isEmpty())
  {
    auto orphanIndex = this->orphanFrames.dequeue();

    auto fd = this->frames.find(orphanIndex);
    if (fd != this->frames.end())
    {
      fd->camera = vtkSmartPointer<vtkMaptkCamera>::New();
      fd->camera->SetCamera(camera);
      fd->camera->Update();

      this->UI.worldView->addCamera(fd->id, fd->camera);
      if (fd->id == this->activeCameraIndex)
      {
        this->UI.worldView->setActiveCamera(fd->id);
        this->updateCameraView();
      }

      return;
    }
  }

  // Add the camera to the end
  unsigned int lastFrameId =
    this->frames.isEmpty() ? 0 : this->frames.lastKey();
  this->addFrame(camera, lastFrameId + 1);
}

//-----------------------------------------------------------------------------
void MainWindowPrivate::addImage(QString const& imagePath)
{
  // TODO: Create/manage image list video source
}

//-----------------------------------------------------------------------------
void MainWindowPrivate::addVideoSource(
  kwiver::vital::config_block_sptr const& config,
  QString const& videoPath)
{
  // Save the configuration so independent video sources can be created for tools
  if (this->project)
  {
    this->project->config->merge_config(config);
  }
  this->videoPath = videoPath;
  this->freestandingConfig->merge_config(config);

  // Close the existing video source if it exists
  if(this->videoSource)
  {
    this->videoSource->close();
  }

  kwiver::vital::algo::video_input::
    set_nested_algo_configuration("video_reader", config, this->videoSource);

  videoImporter.setData(config, stdString(videoPath), this->localGeoCs);

  try
  {
    if (this->videoSource)
    {
      this->videoSource->open(stdString(videoPath));
    }

    // Set the skip value if present
    // TODO: fix kwiver so this is done with an adapter and it not in the video source
    if (this->videoSource)
    {
      if(config->has_value("video_reader:vidl_ffmpeg:output_nth_frame"))
      {
        this->advanceInterval =
          config->get_value<int>("video_reader:vidl_ffmpeg:output_nth_frame");
      }
      else if(config->has_value("video_reader:splice:output_nth_frame"))
      {
        this->advanceInterval =
          config->get_value<int>("video_reader:splice:output_nth_frame");
      }
    }

    foreach (auto const& tool, this->tools)
    {
      tool->setEnabled(false);
    }

    videoImporter.start();
  }
  catch (kwiver::vital::file_not_found_exception const& e)
  {
    qWarning() << e.what();
    this->videoSource->close();
    this->videoSource.reset();
  }
}

//-----------------------------------------------------------------------------
void MainWindowPrivate::addMaskSource(
  kwiver::vital::config_block_sptr const& config, QString const& maskPath)
{
  // Save the configuration so independent video sources can be created for
  // tools
  if (this->project)
  {
    this->project->config->merge_config(config);
  }
  this->maskPath = maskPath;
  this->freestandingConfig->merge_config(config);
}

//-----------------------------------------------------------------------------
void MainWindowPrivate::addFrame(
  kwiver::vital::camera_perspective_sptr const& camera, int id)
{
  if (this->frames.find(id) != this->frames.end())
  {
    qWarning() << "Frame " << id << " already exists.";
    return;
  }

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
  else
  {
    this->orphanFrames.enqueue(id);
  }

  this->frames.insert(id, cd);

  // When the first camera is added, show it immediately and reset the camera
  // view, enable slideshow controls, and set the frame number to one.
  if (this->frames.count() == 1)
  {
    this->UI.actionSlideshowPlay->setEnabled(true);
    this->UI.camera->setEnabled(true);
    this->UI.cameraSpin->setEnabled(true);

    this->setActiveCamera(1);
    this->UI.cameraView->resetView();
  }

  unsigned int lastFrameId =
    this->frames.isEmpty() ? 1 : this->frames.lastKey();
  this->UI.camera->setRange(1, lastFrameId);
  this->UI.cameraSpin->setRange(1, lastFrameId);
}

//-----------------------------------------------------------------------------
void MainWindowPrivate::updateFrames(
  std::shared_ptr<kwiver::vital::metadata_map::map_metadata_t> mdMap)
{
  this->videoMetadataMap = *mdMap;

  this->UI.metadata->updateMetadata(mdMap);

  if (this->project &&
      this->project->config->has_value("output_krtd_dir"))
  {
    qWarning() << "Loading project cameras with frames.count = " << this->frames.count();
    for (auto const& frame : this->frames)
    {
      auto frameName = qtString(this->getFrameName(frame.id)) + ".krtd";

      try
      {
        auto const& camera = kwiver::vital::read_krtd_file(
          kvPath(frameName), kvPath(this->project->cameraPath));

        // Add camera to scene
        this->updateCamera(frame.id, camera);
      }
      catch (...)
      {
        qWarning() << "failed to read camera file " << frameName
                   << " from " << this->project->cameraPath;
      }
    }
  }
  else
  {
#define GET_K_CONFIG(type, name) \
  this->freestandingConfig->get_value<type>(bc + #name, K_def.name())

    kwiver::vital::simple_camera_intrinsics K_def;
    const std::string bc = "video_reader:base_camera:";
    auto K = std::make_shared<kwiver::vital::simple_camera_intrinsics>(
      GET_K_CONFIG(double, focal_length),
      GET_K_CONFIG(kwiver::vital::vector_2d, principal_point),
      GET_K_CONFIG(double, aspect_ratio),
      GET_K_CONFIG(double, skew));

    auto baseCamera = kwiver::vital::simple_camera_perspective();
    baseCamera.set_intrinsics(K);

    kwiver::vital::camera_map::map_camera_t camMap;
    if (videoMetadataMap.size() > 0)
    {
      std::map<kwiver::vital::frame_id_t, kwiver::vital::metadata_sptr> mdMap;
      for (auto const& mdIter: this->videoMetadataMap)
      {
        // TODO: just using first element of metadata vector for now
        mdMap[mdIter.first] = mdIter.second[0];
      }

      bool init_cams_with_metadata =
        this->freestandingConfig->get_value<bool>(
          "initialize_cameras_with_metadata", true);

      if (init_cams_with_metadata)
      {
        auto im = this->videoSource->frame_image();

        bool init_intrinsics_with_metadata =
          this->freestandingConfig->get_value<bool>(
            "initialize_intrinsics_with_metadata", true);
        if (init_intrinsics_with_metadata)
        {
          kwiver::maptk::set_intrinsics_from_metadata(baseCamera, mdMap, im);
        }

        camMap = kwiver::maptk::initialize_cameras_with_metadata(
          mdMap, baseCamera, this->localGeoCs);
      }
    }

    this->updateCameras(std::make_shared<kwiver::vital::simple_camera_map>(camMap));
  }

  //find depth map paths
  if (this->project &&
      this->project->config->has_value("output_depth_dir"))
  {
    foreach(auto & frame, this->frames)
    {
      auto depthName = qtString(this->getFrameName(frame.id)) + ".vti";
      auto depthMapPath = QDir{this->project->depthPath}.filePath(depthName);
      QFileInfo check_file(depthMapPath);
      if (check_file.exists() && check_file.isFile())
      {
        frame.depthMapPath = depthMapPath;
      }
    }
  }

  this->UI.worldView->initFrameSampling(this->frames.size());

  if (this->project){
    for (auto const& tool : this->tools)
    {
      tool->setEnabled(true);
    }
  }

}

//-----------------------------------------------------------------------------
kwiver::vital::camera_map_sptr MainWindowPrivate::cameraMap() const
{
  kwiver::vital::camera_map::map_camera_t map;

  for (auto cd : this->frames)
  {
    if (cd.camera)
    {
      map.insert(std::make_pair(static_cast<kwiver::vital::frame_id_t>(cd.id),
                                cd.camera->GetCamera()));
    }
  }

  return std::make_shared<kwiver::vital::simple_camera_map>(map);
}

//-----------------------------------------------------------------------------
void MainWindowPrivate::updateCameras(
  kwiver::vital::camera_map_sptr const& cameras)
{
  auto allowExport = false;

  std::set<kwiver::vital::frame_id_t> updated_frame_ids;
  foreach (auto const& iter, cameras->cameras())
  {
    using kwiver::vital::camera_perspective;
    auto cam_ptr = std::dynamic_pointer_cast<camera_perspective>(iter.second);
    if (updateCamera(iter.first, cam_ptr))
    {
      updated_frame_ids.insert(iter.first);
      allowExport = allowExport || iter.second;
    }
  }

  for (auto &f : this->frames)
  {
    auto fid = f.id;
    if (updated_frame_ids.find(fid) != updated_frame_ids.end())
    {
      continue;
    }
    if (f.camera)
    {
      f.camera = NULL;
      this->UI.worldView->removeCamera(fid);
    }
  }

  this->UI.actionExportCameras->setEnabled(allowExport);
}

//-----------------------------------------------------------------------------
bool MainWindowPrivate::updateCamera(kwiver::vital::frame_id_t frame,
                                     kwiver::vital::camera_perspective_sptr cam)
{
  auto fr = this->frames.find(frame);
  if (fr == this->frames.end() || !cam)
  {
    return false;
  }
  if (!fr->camera)
  {
    fr->camera = vtkSmartPointer<vtkMaptkCamera>::New();
    this->UI.worldView->addCamera(fr->id, fr->camera);
  }
  fr->camera->SetCamera(cam);
  fr->camera->Update();

  // Remove from orphanFrames if needed.
  auto orphanIndex = orphanFrames.indexOf(frame);
  if (orphanIndex >= 0)
  {
    orphanFrames.removeAt(orphanIndex);
  }

  if (fr->id == this->activeCameraIndex)
  {
    this->UI.worldView->setActiveCamera(fr->id);
    this->updateCameraView();
  }

  return true;
}

//-----------------------------------------------------------------------------
void MainWindowPrivate::setActiveCamera(int id)
{
  //if only keyframes are to be displayed in the camera view
  bool only_keyframes = this->UI.actionKeyframesOnly->isChecked();
  bool next_frame_found = false;

  if (id >= this->activeCameraIndex)
  { //positive movement in sequence
    //find the next keyframe in the sequence
    int lastFrameId =
      this->frames.isEmpty() ? 1 : this->frames.lastKey();
    while (id <= lastFrameId)
    {
      if (only_keyframes)
      {
        auto fd = std::dynamic_pointer_cast<kwiver::vital::feature_track_set_frame_data>(
          tracks->frame_data(id));

        if (fd && fd->is_keyframe)
        {
          next_frame_found = true;
          break;
        }
      }
      else
      {
        if (id == 1 || (id - 1)%this->advanceInterval == 0)
        {
          next_frame_found = true;
          break;
        }
      }
      ++id;
    }
  }
  else
  { //going backward in sequence
    //find the previous keyframe in the sequence
    while (id >= 1)
    {
      if (only_keyframes)
      {
        auto fd = std::dynamic_pointer_cast<kwiver::vital::feature_track_set_frame_data>(
          tracks->frame_data(id));

        if (fd && fd->is_keyframe)
        {
          next_frame_found = true;
          break;
        }
      }
      else
      {
        if (id == 1 || (id - 1)%this->advanceInterval ==0)
        {
          next_frame_found = true;
          break;
        }
      }
      --id;
    }
  }
  if (!next_frame_found)
  {
    // There was not a keyframe to move to in the direction we're going.
    // So set the active camera back to what it was.
    this->UI.camera->setValue(this->activeCameraIndex);
    this->UI.cameraSpin->setValue(this->activeCameraIndex);
    return;
  }

  auto oldSignalState = this->UI.camera->blockSignals(true);
  this->UI.camera->setValue(id);
  this->UI.camera->blockSignals(oldSignalState);
  this->UI.cameraSpin->setValue(id);

  this->activeCameraIndex = id;
  this->UI.worldView->setActiveCamera(id);

  this->updateCameraView();

  //load from memory if cached
  if (id == this->activeDepthFrame)
  {
    this->depthReader->SetFileName("");
    this->depthFilter->RemoveAllInputConnections(0);
    this->depthFilter->SetInputData(this->activeDepth);

    this->UI.depthMapView->setValidDepthInput(true);
    this->UI.worldView->setValidDepthInput(true);

    auto fr = this->frames.find(id);
    if (fr != this->frames.end())
    {
      this->depthFilter->SetCamera(fr->camera);
    }
    this->UI.worldView->updateDepthMap();
    this->UI.depthMapView->updateView(true);
    this->UI.actionExportDepthPoints->setEnabled(true);
  }
  else // load from file
  {
    auto fr = this->frames.find(id);
    if (fr != this->frames.end())
    {
      if (!fr->depthMapPath.isEmpty())
      {
        this->loadDepthMap(fr->depthMapPath);
      }
    }
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

  auto activeFrame = this->frames.find(this->activeCameraIndex);

  auto fid = activeFrame.key();

  if (activeFrame == this->frames.end())
  {
    this->loadEmptyImage(0);
    this->UI.cameraView->clearLandmarks();
    return;
  }

  // Show camera image
  this->loadImage(*activeFrame);


  if (!activeFrame->camera)
  {
    // Can't show landmarks or residuals with no camera
    this->UI.cameraView->clearLandmarks();
    this->UI.cameraView->clearResiduals();
    return;
  }

  // Show landmarks
  QHash<kwiver::vital::track_id_t, kwiver::vital::vector_2d> landmarkPoints;
  this->UI.cameraView->clearLandmarks();
  if (this->landmarks)
  {
    // Map landmarks to camera space
    auto const& landmarks = this->landmarks->landmarks();
    foreach (auto const& lm, landmarks)
    {
      double pp[2];
      if (activeFrame->camera->ProjectPoint(lm.second->loc(), pp))
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
  this->UI.cameraView->render();
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
  imageImport->SetNumberOfScalarComponents(static_cast<int>(img.depth()));
  imageImport->SetWholeExtent(0, static_cast<int>(img.width())-1,
                              0, static_cast<int>(img.height())-1, 0, 0);
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
  return frameName(frameId, this->videoMetadataMap);
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
      this->UI.cameraView->setImagePath(
        qtString(this->getFrameName(frame.id)));

      // Set image on views
      auto const size = QSize(dimensions[0], dimensions[1]);
      this->UI.cameraView->setImageData(imageData, size);
      this->UI.worldView->setImageData(imageData, size);

      // Update metadata view
      if (this->videoMetadataMap.empty())
      {
        this->UI.metadata->updateMetadata(kwiver::vital::metadata_vector{});
      }
      else
      {
        auto mdi = --(this->videoMetadataMap.upper_bound(frame.id));
        this->UI.metadata->updateMetadata(mdi->second);
      }
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
  if (!QFileInfo{imagePath}.isFile())
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

  this->depthFilter->RemoveAllInputs();
  this->depthFilter->SetInputConnection(this->depthReader->GetOutputPort());

  this->depthReader->SetFileName(qPrintable(imagePath));

  this->UI.depthMapView->setValidDepthInput(true);
  this->UI.worldView->setValidDepthInput(true);

  auto activeFrame = this->frames.find(this->activeCameraIndex);
  if (activeFrame != this->frames.end())
  {
    this->depthFilter->SetCamera(activeFrame->camera);
  }
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

  // Connect actions
  if (tool)
  {
    QObject::connect(this->UI.actionCancelComputation, SIGNAL(triggered()),
                     tool, SLOT(cancel()));
    QObject::connect(this->UI.actionCancelComputation, SIGNAL(triggered()),
                     project.data(), SLOT(write()));
    QObject::connect(this->UI.actionQuit, SIGNAL(triggered()),
                     tool, SLOT(cancel()));
    QObject::connect(tool, SIGNAL(completed()),
                     project.data(), SLOT(write()));
  }

  auto const enableTools = !tool;
  auto const enableCancel = tool && tool->isCancelable();
  foreach (auto const& tool, this->tools)
  {
    tool->setEnabled(enableTools);
  }
  this->UI.actionCancelComputation->setEnabled(enableCancel);
  this->UI.actionOpenProject->setEnabled(enableTools);
  // FIXME disable import actions
}

//-----------------------------------------------------------------------------
void MainWindowPrivate::updateProgress(QObject* object,
                                       const QString& description,
                                       int value)
{
  QString desc = description;
  desc.replace('&', "");
  int taskId = -1;
  if (!this->progressIds.contains(object))
  {
    taskId = this->UI.progressWidget->addTask(desc, 0, 0, 0);
    this->progressIds.insert(object, taskId);
    return;
  }
  else
  {
    taskId = this->progressIds.value(object);
  }

  this->UI.progressWidget->setTaskText(taskId, desc);
  this->UI.progressWidget->setProgressValue(taskId, value);
  switch(value)
  {
    case 0:
    {
      this->UI.progressWidget->setProgressRange(taskId, 0, 0);
      break;
    }
    case 100:
    {
      this->UI.progressWidget->removeTask(taskId);
      this->progressIds.remove(object);
      break;
    }
    default:
    {
      this->UI.progressWidget->setProgressRange(taskId, 0, 100);
      break;
    }
  }
}

//END MainWindowPrivate

///////////////////////////////////////////////////////////////////////////////

//BEGIN MainWindow

//-----------------------------------------------------------------------------
MainWindow::MainWindow(QWidget* parent, Qt::WindowFlags flags)
  : QMainWindow(parent, flags), d_ptr(new MainWindowPrivate(this))
{
  QTE_D();

  // Set up UI
  d->UI.setupUi(this);
  d->AM.setupActions(d->UI, this);

  d->toolMenu = d->UI.menuCompute;
  d->toolSeparator =
    d->UI.menuCompute->insertSeparator(d->UI.actionCancelComputation);

  d->addTool(new TrackFeaturesTool(this), this);
  d->addTool(new TriangulateTool(this), this);
  d->addTool(new BundleAdjustTool(this), this);
  d->addTool(new SaveFrameTool(this), this);
  d->addTool(new ComputeDepthTool(this), this);

  d->toolMenu = d->UI.menuAdvanced;
  d->toolSeparator =
    d->UI.menuAdvanced->addSeparator();
  d->addTool(new TrackFeaturesSprokitTool(this), this);
  d->addTool(new NeckerReversalTool(this), this);
  d->addTool(new TrackFilterTool(this), this);
  d->addTool(new InitCamerasLandmarksTool(this), this);
  d->addTool(new SaveKeyFrameTool(this), this);
  d->addTool(new CanonicalTransformTool(this), this);

  d->UI.menuView->addSeparator();
  d->UI.menuView->addAction(d->UI.cameraViewDock->toggleViewAction());
  d->UI.menuView->addAction(d->UI.cameraSelectorDock->toggleViewAction());
  d->UI.menuView->addAction(d->UI.metadataDock->toggleViewAction());
  d->UI.menuView->addAction(d->UI.depthMapViewDock->toggleViewAction());

  d->UI.playSlideshowButton->setDefaultAction(d->UI.actionSlideshowPlay);
  d->UI.loopSlideshowButton->setDefaultAction(d->UI.actionSlideshowLoop);

  connect(d->UI.actionQuit, SIGNAL(triggered()), qApp, SLOT(quit()));

  connect(d->UI.actionNewProject, SIGNAL(triggered()),
          this, SLOT(newProject()));
  connect(d->UI.actionOpenProject, SIGNAL(triggered()),
          this, SLOT(openProject()));
  connect(d->UI.actionImportImagery, SIGNAL(triggered()),
          this, SLOT(openImagery()));
  connect(d->UI.actionImportMasks, SIGNAL(triggered()),
          this, SLOT(openMaskImagery()));
  connect(d->UI.actionImportCameras, SIGNAL(triggered()),
          this, SLOT(openCameras()));
  connect(d->UI.actionImportTracks, SIGNAL(triggered()),
          this, SLOT(openTracks()));
  connect(d->UI.actionImportLandmarks, SIGNAL(triggered()),
          this, SLOT(openLandmarks()));

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
  connect(d->UI.slideSpeed, SIGNAL(valueChanged(int)),
          this, SLOT(setSlideSpeed(int)));

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

  this->setSlideSpeed(d->UI.slideSpeed->value());

#ifdef VTKWEBGLEXPORTER
  d->UI.actionWebGLScene->setVisible(true);
  connect(d->UI.actionWebGLScene, SIGNAL(triggered(bool)),
          this, SLOT(saveWebGLScene()));
#endif

  // Set up UI persistence and restore previous state
  auto const sdItem = new qtUiState::Item<int, QSlider>(
    d->UI.slideSpeed, &QSlider::value, &QSlider::setValue);
  d->uiState.map("SlideSpeed", sdItem);

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

  // Set up the progress widget
  d->UI.progressWidget->setAutoHide(true);

  // Antialiasing
  connect(d->UI.actionAntialiasing, SIGNAL(toggled(bool)),
          this, SLOT(enableAntiAliasing(bool)));

  // Common ROI
  // Unitil the bounding box is initialized, the bounds are going to be
  // [VTK_DOUBLE_MIN, VTK_DOUBLE_MAX] in all directions.
  d->UI.worldView->setROI(d->roi.GetPointer());
}

//-----------------------------------------------------------------------------
MainWindow::~MainWindow()
{
  QTE_D();
  d->uiState.save();
}

//-----------------------------------------------------------------------------
void MainWindow::openProject()
{
  auto const path = QFileDialog::getOpenFileName(
    this, "Open Project", QString(),
    "Project configuration files (*.conf);;"
    "All Files (*)");

  if (!path.isEmpty())
  {
    this->loadProject(path);
  }
}

//-----------------------------------------------------------------------------
void MainWindow::openImagery()
{
  static auto const imageFilters =
    makeFilters(supportedImageExtensions().toList());
  static auto const videoFilters =
    makeFilters(supportedVideoExtensions().toList());

  // TODO: Add image filters back once that is supported again.
  auto const paths = QFileDialog::getOpenFileNames(
    this, "Open Imagery", QString(),
    "All Supported Files (" + videoFilters + ");;"
    "Video files (" + videoFilters + ");;"
    "All Files (*)");

  for (auto const& path : paths)
  {
    this->loadImagery(path);
  }
}

//-----------------------------------------------------------------------------
void MainWindow::openMaskImagery()
{
  static auto const imageFilters =
    makeFilters(supportedImageExtensions().toList());
  static auto const videoFilters =
    makeFilters(supportedVideoExtensions().toList());

  // TODO: Add image filters back once that is supported again.
  auto const paths = QFileDialog::getOpenFileNames(
    this, "Open Mask Imagery", QString(),
    "All Supported Files (" + videoFilters + ");;"
    "Video files (" + videoFilters + ");;"
    "All Files (*)");

  for (auto const& path : paths)
  {
    this->loadMaskImagery(path);
  }
}

//-----------------------------------------------------------------------------
void MainWindow::openCameras()
{
  auto const paths = QFileDialog::getOpenFileNames(
    this, "Open Cameras", QString(),
    "Camera files (*.krtd);;"
    "All Files (*)");

  for (auto const& path : paths)
  {
    this->loadCamera(path);
  }
}

//-----------------------------------------------------------------------------
void MainWindow::openTracks()
{
  auto const paths = QFileDialog::getOpenFileNames(
    this, "Open Feature Tracks", QString(),
    "Feature track files (*.txt);;"
    "All Files (*)");

  for (auto const& path : paths)
  {
    this->loadTracks(path);
  }
}

//-----------------------------------------------------------------------------
void MainWindow::openLandmarks()
{
  auto const paths = QFileDialog::getOpenFileNames(
    this, "Open Landmarks", QString(),
    "Landmark files (*.ply);;"
    "All Files (*)");

  for (auto const& path : paths)
  {
    this->loadLandmarks(path);
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

    d->project.reset(new Project{dirname});

    if (d->videoSource)
    {
      d->project->config->merge_config(d->freestandingConfig);
      d->project->videoPath = d->videoPath;
      d->project->maskPath = d->maskPath;
    }

    saveCameras(d->project->cameraPath);
    d->project->config->set_value("output_krtd_dir", kvPath(
      d->project->getContingentRelativePath(d->project->cameraPath)));

    if (!d->localGeoCs.origin().is_empty() &&
        !d->project->geoOriginFile.isEmpty())
    {
      saveGeoOrigin(d->project->geoOriginFile);
    }

    d->project->write();
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

  QScopedPointer<Project> project{new Project};
  if (!project->read(path))
  {
    qWarning() << "Failed to load project from" << path; // TODO dialog?
    return;
  }
  d->project.reset(project.take());

  // Set the current working directory to the project directory
  if (!QDir::setCurrent(d->project->workingDir.absolutePath()))
  {
    qWarning() << "Unable to set current working directory to "
      << "project directory: " << d->project->workingDir.absolutePath();
  }

  // Get the video and mask sources
  if (d->project->config->has_value("video_reader:type"))
  {
    d->addVideoSource(d->project->config, d->project->videoPath);
  }
  if (d->project->config->has_value("mask_reader:type"))
  {
    d->addMaskSource(d->project->config, d->project->maskPath);
  }

  // Load tracks
  if (d->project->config->has_value("input_track_file") ||
      d->project->config->has_value("output_tracks_file"))
  {
    this->loadTracks(d->project->tracksPath);
  }

  // Load landmarks
  if (d->project->config->has_value("output_ply_file"))
  {
    this->loadLandmarks(d->project->landmarksPath);
  }

  // Cameras and depth maps are loaded after video importer is done

#ifdef VTKWEBGLEXPORTER
  d->UI.actionWebGLScene->setEnabled(true);
#endif

  // Load volume
  if (d->project->config->has_value("volume_file"))
  {
    d->UI.worldView->loadVolume(d->project->volumePath,
                                d->project->cameraPath,
                                d->project->videoPath);
  }

  if (d->project->config->has_value("geo_origin_file"))
  {
    if (QFileInfo{d->project->geoOriginFile}.isFile())
    {
      kwiver::maptk::read_local_geo_cs_from_file(
        d->localGeoCs, stdString(d->project->geoOriginFile));
    }
    else
    {
      qWarning() << "Failed to open geo origin file "
        << d->project->geoOriginFile << ". File does not exist.";
    }
  }

  d->UI.worldView->queueResetView();

  foreach (auto const& tool, d->tools)
  {
    tool->setEnabled(true);
  }

  d->setActiveCamera(d->activeCameraIndex);
}

//-----------------------------------------------------------------------------
void MainWindow::loadImagery(QString const& path)
{
  static auto const imageExtensions = supportedImageExtensions();
  static auto const videoExtensions = supportedVideoExtensions();

  auto const ext = QFileInfo{path}.suffix().toLower();
  if (imageExtensions.contains(ext))
  {
    this->loadImage(path);
  }
  else if (videoExtensions.contains(ext))
  {
    // TODO: Handle [selection of] multiple videos better
    this->loadVideo(path);
  }
  else
  {
    qWarning() << "Don't know how to read file" << path
               << "(unrecognized extension)";
  }
}

//-----------------------------------------------------------------------------
void MainWindow::loadMaskImagery(QString const& path)
{
  static auto const imageExtensions = supportedImageExtensions();
  static auto const videoExtensions = supportedVideoExtensions();

  auto const ext = QFileInfo{path}.suffix().toLower();
  if (imageExtensions.contains(ext))
  {
    this->loadMaskImage(path);
  }
  else if (videoExtensions.contains(ext))
  {
    // TODO: Handle [selection of] multiple videos better
    this->loadMaskVideo(path);
  }
  else
  {
    qWarning() << "Don't know how to read file" << path
               << "(unrecognized extension)";
  }
}

//-----------------------------------------------------------------------------
void MainWindow::loadImage(QString const& path)
{
  QTE_D();
  d->addImage(path);
}

//-----------------------------------------------------------------------------
void MainWindow::loadVideo(QString const& path)
{
  QTE_D();

  auto config = readConfig(imageConfigForPath(path, "image"));
  if (d->project)
  {
    d->project->config->merge_config(config);
    d->project->videoPath = path;
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

  if (d->project)
  {
    saveCameras(d->project->cameraPath);
    d->project->config->set_value("output_krtd_dir", kvPath(
      d->project->getContingentRelativePath(d->project->cameraPath)));

    if (!d->localGeoCs.origin().is_empty() &&
        !d->project->geoOriginFile.isEmpty())
    {
      saveGeoOrigin(d->project->geoOriginFile);
    }

    d->project->write();
  }

  d->UI.worldView->queueResetView();
}

//-----------------------------------------------------------------------------
void MainWindow::loadMaskImage(QString const& path)
{
  // TODO
}

//-----------------------------------------------------------------------------
void MainWindow::loadMaskVideo(QString const& path)
{
  QTE_D();

  auto config = readConfig(imageConfigForPath(path, "mask"));
  if (d->project)
  {
    d->project->config->merge_config(config);
    d->project->maskPath = path;
  }

  try
  {
    d->addMaskSource(config, path);
  }
  catch (std::exception const& e)
  {
    QMessageBox::critical(
      this, "Error loading video\n",
      e.what());
  }

  if (d->project)
  {
    d->project->write();
  }
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

    typedef std::unique_ptr<kwiver::vital::track_set_implementation> tsi_uptr;

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

        auto tks_temp =
          std::make_shared<kwiver::vital::feature_track_set>(
            tsi_uptr(new kwiver::arrows::core::frame_index_track_set_impl(new_tracks)));
        tks_temp->set_frame_data(tracks->all_frame_data());
        tracks = tks_temp;
      }
      else
      {
        auto tks_temp = std::make_shared<kwiver::vital::feature_track_set>(
          tsi_uptr(new kwiver::arrows::core::frame_index_track_set_impl(tracks->tracks())));
        tks_temp->set_frame_data(tracks->all_frame_data());
        tracks = tks_temp;
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
      d->UI.actionKeyframesOnly->setEnabled(!tracks->tracks().empty());
    }
  }
  catch (std::exception const& e)
  {
    qWarning() << "failed to read tracks from" << path << " with error: " << e.what();
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

    if (writeToProject && d->project)
    {
      d->project->config->set_value("output_ply_file", kvPath(
        d->project->getContingentRelativePath(path)));
    }
  }
  catch (...)
  {
    auto const msg =
      QString("An error occurred while exporting landmarks to \"%1\". "
              "The output file may not have been written correctly.");
    QMessageBox::critical(
      this, "Export error", msg.arg(d->project->landmarksPath));
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

    if (writeToProject && d->project)
    {
      d->project->config->set_value("output_tracks_file", kvPath(
        d->project->getContingentRelativePath(path)));
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

  auto out = QHash<QString, kwiver::vital::camera_perspective_sptr>();
  auto willOverwrite = QStringList();

  for (auto cd : d->frames)
  {
    if (cd.camera)
    {
      auto const camera = cd.camera->GetCamera();
      if (camera)
      {
        auto cameraName = qtString(d->getFrameName(cd.id)) + ".krtd";
        auto const filepath = QDir{path}.filePath(cameraName);
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

    QAbstractButton* myOverwrite = mb.addButton("&Overwrite", QMessageBox::AcceptRole);
    mb.setDetailedText("The following file(s) will be overwritten:\n  " +
                       willOverwrite.join("  \n"));

    mb.exec();
    if (mb.clickedButton() != myOverwrite)
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
      auto cam_ptr =
        std::dynamic_pointer_cast<kwiver::vital::camera_perspective>(iter.value());
      kwiver::vital::write_krtd_file(*cam_ptr, kvPath(iter.key()));
    }
    catch (...)
    {
      errors.append(iter.key());
    }
  }

  if (writeToProject && d->project)
  {
    d->project->config->set_value("output_krtd_dir", kvPath(
      d->project->getContingentRelativePath(path)));
  }

  if (!errors.isEmpty())
  {
    auto const msg =
      QString("Error(s) occurred while exporting cameras to \"%1\". "
              "One or more output files may not have been written correctly.");

    QMessageBox mb(QMessageBox::Critical, "Export error",
                   msg.arg(d->project->cameraPath), QMessageBox::Ok, this);

    mb.setDetailedText("Error writing the following file(s):\n  " +
                       errors.join("  \n"));

    mb.exec();
  }
}

//-----------------------------------------------------------------------------
void MainWindow::saveDepthImage(QString const& path)
{
  QTE_D();

  if (!d->activeDepth || d->activeDepthFrame < 1)
  {
    return;
  }

  auto filename = qtString(d->getFrameName(d->activeDepthFrame)) + ".vti";

  if (!QDir(path).exists())
  {
    QDir().mkdir(path);
  }


  vtkNew<vtkXMLImageDataWriter> writerI;
  auto const filepath = QDir{path}.filePath(filename);
  writerI->SetFileName(qPrintable(filepath));
  writerI->AddInputDataObject(d->activeDepth.Get());
  writerI->SetDataModeToBinary();
  writerI->Write();

  auto activeFrame = d->frames.find(d->activeDepthFrame);
  if (activeFrame != d->frames.end())
  {
    activeFrame->depthMapPath = filepath;
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
    d->project->config->set_value("depthmaps_images_file",
      kvPath(d->project->getContingentRelativePath(path)));
    d->project->write();
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

  d->project->config->set_value("geo_origin_file", kvPath(
    d->project->getContingentRelativePath(path)));
  kwiver::maptk::write_local_geo_cs_to_file(d->localGeoCs, stdString(path));
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
    d->project->volumePath = d->project->getContingentRelativePath(path);
    d->project->config->set_value("volume_file",
      kvPath(d->project->volumePath));
    d->project->write();
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
void MainWindow::setSlideSpeed(int speed)
{
  QTE_D();

  static auto const ttFormat =
    QString("%1 (%2)").arg(d->UI.slideSpeed->toolTip());

  auto dt = QString("Unconstrained");
  auto delay = 0.0;
  if (speed < 60)
  {
    auto const de = static_cast<double> (speed) * 0.1;
    auto const fps = qPow(2.0, de);
    delay = 1e3 / fps;
    dt = QString("%1 / sec").arg(fps, 0, 'g', 2);
  }
  d->slideTimer.setInterval(delay);
  d->UI.slideSpeed->setToolTip(ttFormat.arg(dt));
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

  int lastFrameId = d->frames.isEmpty() ? 1 : d->frames.lastKey();
  if (id < 1 || id > lastFrameId)
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
      tool->setVideoPath(stdString(d->videoPath));
      tool->setMaskPath(stdString(d->maskPath));
      tool->setConfig(d->project->config);
      if (!d->frames.empty())
      {
        tool->setLastFrame(static_cast<int>(d->frames.lastKey()));
      }

      if (!tool->execute())
      {
        d->setActiveTool(0);
      }
      else
      {
        // Initialize the progress bar
        d->updateProgress(tool, tool->description(), 0);
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
    acceptToolResults(d->activeTool->data(), true);
    saveToolResults();
    // Signal tool execution as complete to the progress widget
    d->updateProgress(d->activeTool,
                      d->activeTool->description(),
                      100);
  }
  d->setActiveTool(0);
}

//-----------------------------------------------------------------------------
void MainWindow::acceptToolResults(std::shared_ptr<ToolData> data, bool isFinal)
{
  QTE_D();
  // if all the update variables are Null then trigger a GUI update after
  // extracting the data otherwise we've already triggered an update that
  // hasn't happened yet, so don't trigger another
  bool updateNeeded = !d->toolUpdateCameras &&
                      !d->toolUpdateLandmarks &&
                      !d->toolUpdateTracks &&
                      !d->toolUpdateDepth &&
                      d->toolUpdateActiveFrame < 0;

  if (d->activeTool)
  {
    auto const outputs = d->activeTool->outputs();

    d->toolUpdateCameras = NULL;
    d->toolUpdateLandmarks = NULL;
    d->toolUpdateTracks = NULL;
    d->toolUpdateActiveFrame = -1;
    d->toolUpdateDepth = NULL;
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
    if (outputs.testFlag(AbstractTool::Depth))
    {
      d->toolUpdateDepth = data->active_depth;
    }
    if (outputs.testFlag(AbstractTool::ActiveFrame))
    {
      d->toolUpdateActiveFrame = static_cast<int>(data->activeFrame);
    }
    // Update tool progress
    d->updateProgress(d->activeTool,
                      d->activeTool->description(),
                      d->activeTool->progress());
  }

  if (isFinal)
  {
    updateToolResults();  //force immediate update on tool finish so we ensure update before saving
  }
  else if(updateNeeded)
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
      saveCameras(d->project->cameraPath);
    }
    if (outputs.testFlag(AbstractTool::Landmarks))
    {
      saveLandmarks(d->project->landmarksPath);
    }
    if (outputs.testFlag(AbstractTool::Tracks))
    {
      saveTracks(d->project->tracksPath);
    }

    if (!d->localGeoCs.origin().is_empty() &&
        !d->project->geoOriginFile.isEmpty())
    {
      saveGeoOrigin(d->project->geoOriginFile);
    }
    if (outputs.testFlag(AbstractTool::Depth))
    {
      saveDepthImage(d->project->depthPath);
      d->project->config->set_value("output_depth_dir", kvPath(
        d->project->getContingentRelativePath(d->project->depthPath)));
    }

    d->project->write();
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
    foreach (auto const& track, d->tracks->tracks())
    {
      d->UI.cameraView->addFeatureTrack(*track);
    }
    d->UI.actionExportTracks->setEnabled(
        d->tracks && d->tracks->size());

    d->UI.actionShowMatchMatrix->setEnabled(!d->tracks->tracks().empty());
    d->UI.actionKeyframesOnly->setEnabled(!d->tracks->tracks().empty());
    d->toolUpdateTracks = NULL;
  }
  if (d->toolUpdateDepth)
  {
    d->activeDepth = d->toolUpdateDepth;
    d->activeDepthFrame = d->toolUpdateActiveFrame;

    d->toolUpdateDepth = NULL;
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
void MainWindow::addFrame(int frame)
{
  QTE_D();

  d->addFrame(nullptr, frame);
}

//-----------------------------------------------------------------------------
void MainWindow::updateFrames(
  std::shared_ptr<kwiver::vital::metadata_map::map_metadata_t> mdMap)
{
  QTE_D();

  d->updateFrames(mdMap);
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

//-----------------------------------------------------------------------------
void MainWindow::updateVideoImportProgress(QString desc, int progress)
{
  QTE_D();

  d->updateProgress(this->sender(), desc, progress);
}

//-----------------------------------------------------------------------------
void MainWindow::enableAntiAliasing(bool enable)
{
  QTE_D();

  d->UI.worldView->enableAntiAliasing(enable);
  d->UI.cameraView->enableAntiAliasing(enable);
  d->UI.depthMapView->enableAntiAliasing(enable);
}

//END MainWindow
