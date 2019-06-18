/*ckwg +29
 * Copyright 2016-2019 by Kitware, Inc.
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

#include "GuiCommon.h"
#include "MainWindow.h"

#include "am_MainWindow.h"
#include "ui_MainWindow.h"

#include "tools/BundleAdjustTool.h"
#include "tools/CanonicalTransformTool.h"
#include "tools/ComputeAllDepthTool.h"
#include "tools/ComputeDepthTool.h"
#include "tools/FuseDepthTool.h"
#include "tools/InitCamerasLandmarksTool.h"
#include "tools/NeckerReversalTool.h"
#include "tools/SaveFrameTool.h"
#include "tools/SaveKeyFrameTool.h"
#include "tools/TrackFeaturesTool.h"
#include "tools/TrackFilterTool.h"
#include "tools/TriangulateTool.h"

#include "AboutDialog.h"
#include "GroundControlPointsHelper.h"
#include "MatchMatrixWindow.h"
#include "Project.h"
#include "RulerHelper.h"
#include "VideoImport.h"
#include "vtkMaptkCamera.h"
#include "vtkMaptkImageDataGeometryFilter.h"
#include "vtkMaptkImageUnprojectDepth.h"

#include <maptk/version.h>
#include <maptk/write_pdal.h>

#include <arrows/core/match_matrix.h>
#include <arrows/core/track_set_impl.h>
#include <arrows/core/transform.h>
#include <vital/algo/video_input.h>
#include <vital/algo/estimate_similarity_transform.h>
#include <vital/io/camera_io.h>
#include <vital/io/landmark_map_io.h>
#include <vital/io/track_set_io.h>
#include <vital/types/camera_perspective.h>
#include <vital/types/local_geo_cs.h>
#include <vital/types/metadata_map.h>
#include <vital/types/sfm_constraints.h>

#include <vtkBox.h>
#include <vtkDoubleArray.h>
#include <vtkImageData.h>
#include <vtkImageReader2.h>
#include <vtkImageReader2Collection.h>
#include <vtkImageReader2Factory.h>
#include <vtkNew.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkXMLImageDataReader.h>
#include <vtkXMLImageDataWriter.h>

#include <qtEnumerate.h>
#include <qtGet.h>
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
kv::path_t kvPath(QString const& s)
{
  return stdString(s);
}

//-----------------------------------------------------------------------------
QString findUserManual()
{
  static auto const name = "telesculptor.html";
  static auto const product = "telesculptor";
  static auto const version = TELESCULPTOR_VERSION;

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
  vtkImageReader2Factory::GetRegisteredReaders(readers);

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
QString makeFilters(QStringList const& extensions)
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
  StateValue(T const& defaultValue = T{})
    : data{defaultValue} {}

  operator T() const { return this->data; }

  StateValue& operator=(T const& newValue)
  {
    this->data = newValue;
    return *this;
  }

  QVariant value() const override
  {
    return QVariant::fromValue(this->data);
  }

  void setValue(QVariant const& newValue) override
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

  void addCamera(kv::camera_perspective_sptr const& camera);
  void addImage(QString const& imagePath);
  void addVideoSource(kv::config_block_sptr const& config,
                      QString const& videoPath);
  void addMaskSource(kv::config_block_sptr const& config,
                     QString const& maskPath);

  void addFrame(kv::camera_perspective_sptr const& camera, int id);
  void updateFrames(std::shared_ptr<kv::metadata_map::map_metadata_t>);

  kv::camera_map_sptr cameraMap() const;
  void updateCameras(kv::camera_map_sptr const&);
  bool updateCamera(kv::frame_id_t frame,
                    kv::camera_perspective_sptr cam);

  std::shared_ptr<std::map<kwiver::vital::frame_id_t, std::string>>
  depthLookup() const;

  void setActiveCamera(int);
  void updateCameraView();

  std::string getFrameName(kv::frame_id_t frame);

  void loadImage(FrameData frame);
  void loadEmptyImage(vtkMaptkCamera* camera);

  void loadDepthMap(QString const& imagePath);

  void setActiveTool(AbstractTool* tool);
  void updateProgress(QObject* object,
                      const QString& description = QString(""),
                      int value = 0);

  void saveGeoOrigin(QString const& path);
  kv::vector_3d centerLandmarks() const;
  void shiftGeoOrigin(kv::vector_3d const& offset);
  std::string roiToString();
  void loadroi(const std::string& roistr);
  void resetActiveDepthMap(int);

  // Member variables
  Ui::MainWindow UI;
  Am::MainWindow AM;
  qtUiState uiState;

  StateValue<QColor>* viewBackgroundColor = nullptr;

  QTimer slideTimer;
  QSignalMapper toolDispatcher;

  QAction* toolSeparator = nullptr;
  QMenu* toolMenu = nullptr;
  AbstractTool* activeTool = nullptr;
  QList<AbstractTool*> tools;
  int toolUpdateActiveFrame = -1;
  kv::camera_map_sptr toolUpdateCameras;
  kv::landmark_map_sptr toolUpdateLandmarks;
  kv::feature_track_set_sptr toolUpdateTracks;
  kv::feature_track_set_changes_sptr toolUpdateTrackChanges;
  vtkSmartPointer<vtkImageData> toolUpdateDepth;
  vtkSmartPointer<vtkStructuredGrid> toolUpdateVolume;
  bool toolSaveDepthFlag = false;

  kv::config_block_sptr freestandingConfig = kv::config_block::empty_config();

  QString videoPath;
  QString maskPath;
  kv::algo::video_input_sptr videoSource;
  kv::algo::video_input_sptr maskSource;
  kv::timestamp currentVideoTimestamp;
  kv::metadata_map_sptr videoMetadataMap =
    std::make_shared<kv::simple_metadata_map>();

  QMap<kv::frame_id_t, FrameData> frames;
  kv::feature_track_set_sptr tracks;
  kv::landmark_map_sptr landmarks;
  vtkSmartPointer<vtkImageData> activeDepth;
  int activeDepthFrame = -1;
  int currentDepthFrame = -1;

  kv::sfm_constraints_sptr sfmConstraints;

  int activeCameraIndex = -1;

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

  // Manual landmarks
  GroundControlPointsHelper* groundControlPointsHelper;
  RulerHelper* rulerHelper;
};

QTE_IMPLEMENT_D_FUNC(MainWindow)

//-----------------------------------------------------------------------------
MainWindowPrivate::MainWindowPrivate(MainWindow* mainWindow)
{
  QObject::connect(&videoImporter, &VideoImport::updated,
                   mainWindow, &MainWindow::addFrame);
  QObject::connect(&videoImporter, &VideoImport::progressChanged,
                   mainWindow, &MainWindow::updateVideoImportProgress);
  QObject::connect(&videoImporter, &VideoImport::completed,
                   mainWindow, &MainWindow::updateFrames);

  sfmConstraints = std::make_shared<kv::sfm_constraints>();
}

//-----------------------------------------------------------------------------
void MainWindowPrivate::saveGeoOrigin(QString const& path)
{
  project->config->set_value("geo_origin_file", kvPath(
    project->getContingentRelativePath(path)));
  kv::write_local_geo_cs_to_file(sfmConstraints->get_local_geo_cs(),
                                 stdString(path));
}

//-----------------------------------------------------------------------------
kv::vector_3d MainWindowPrivate::centerLandmarks() const
{
  std::vector<double> x, y, z;
  x.reserve(this->landmarks->size());
  y.reserve(this->landmarks->size());
  z.reserve(this->landmarks->size());
  for (auto lm : this->landmarks->landmarks())
  {
    auto v = lm.second->loc();
    x.push_back(v[0]);
    y.push_back(v[1]);
    z.push_back(v[2]);
  }
  // compute the median in x and y
  size_t mid = x.size() / 2;
  std::nth_element(x.begin(), x.begin() + mid, x.end());
  std::nth_element(y.begin(), y.begin() + mid, y.end());
  // compute 5th percentile for z
  size_t pct5 = static_cast<size_t>(x.size() * 0.05);
  std::nth_element(z.begin(), z.begin() + pct5, z.end());

  return kv::vector_3d(x[mid], y[mid], z[pct5]);
}

//-----------------------------------------------------------------------------
void MainWindowPrivate::shiftGeoOrigin(kv::vector_3d const& offset)
{
  auto lgcs = sfmConstraints->get_local_geo_cs();
  if (lgcs.origin().is_empty())
  {
    return;
  }
  lgcs.set_origin_altitude(lgcs.origin_altitude() + offset[2]);
  lgcs.set_origin(kv::geo_point(lgcs.origin().location()
                                  + kv::vector_2d(offset[0], offset[1]),
                                lgcs.origin().crs()));
  sfmConstraints->set_local_geo_cs(lgcs);

  if (!sfmConstraints->get_local_geo_cs().origin().is_empty() &&
      !project->geoOriginFile.isEmpty())
  {
    saveGeoOrigin(project->geoOriginFile);
  }

  // shift the ROI
  kv::vector_3d min_pt, max_pt;
  this->roi->GetXMin(min_pt.data());
  this->roi->GetXMax(max_pt.data());
  if (((max_pt - min_pt).array() > 0.0).all())
  {
    min_pt -= offset;
    this->roi->SetXMin(min_pt.data());
    max_pt -= offset;
    this->roi->SetXMax(min_pt.data());
    UI.worldView->setROI(roi.GetPointer(), true);
    project->config->set_value("ROI", roiToString());
  }

  // shift the landmarks
  for (auto lm : this->landmarks->landmarks())
  {
    auto lmd = std::dynamic_pointer_cast<kv::landmark_d>(lm.second);
    if (lmd)
    {
      lmd->set_loc(lm.second->loc() - offset);
    }
  }
  this->UI.worldView->setLandmarks(*landmarks);
  this->UI.cameraView->setLandmarksData(*landmarks);

  // shift the cameras
  auto cameras = this->cameraMap();
  for (auto cam : cameras->cameras())
  {
    auto cam_ptr =
      std::dynamic_pointer_cast<kv::simple_camera_perspective>(cam.second);
    if (cam_ptr)
    {
      cam_ptr->set_center(cam_ptr->center() - offset);
    }
  }
  this->updateCameras(cameras);

  // shift the GCPs
  for (auto gcp : this->groundControlPointsHelper->groundControlPoints())
  {
    gcp.second->set_loc(gcp.second->loc() - offset);
  }
  this->groundControlPointsHelper->pointsReloaded();
}

//-----------------------------------------------------------------------------
void MainWindowPrivate::addTool(AbstractTool* tool, MainWindow* mainWindow)
{
  this->toolMenu->insertAction(this->toolSeparator, tool);

  this->toolDispatcher.setMapping(tool, tool);

  QObject::connect(tool, &AbstractTool::triggered, &this->toolDispatcher,
                   QOverload<>::of(&QSignalMapper::map));
  QObject::connect(tool, &AbstractTool::updated,
                   mainWindow, &MainWindow::acceptToolInterimResults);
  QObject::connect(tool, &AbstractTool::completed,
                   mainWindow, &MainWindow::acceptToolFinalResults);
  QObject::connect(tool, &AbstractTool::failed,
                   mainWindow, &MainWindow::reportToolError);

  tool->setEnabled(false);

  this->tools.append(tool);
}

//-----------------------------------------------------------------------------
void MainWindowPrivate::addCamera(kv::camera_perspective_sptr const& camera)
{
  if (!this->orphanFrames.isEmpty())
  {
    auto orphanIndex = this->orphanFrames.dequeue();

    if (auto* const fd = qtGet(this->frames, orphanIndex))
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
  kv::config_block_sptr const& config,
  QString const& videoPath)
{
  // Save the configuration so independent video sources can be created for
  // tools
  if (this->project)
  {
    this->project->config->merge_config(config);
  }
  this->videoPath = videoPath;
  this->freestandingConfig->merge_config(config);
  // Set video path and config for volume mesh coloring
  this->UI.worldView->setVideoConfig(videoPath, config);

  // Close the existing video source if it exists
  if (this->videoSource)
  {
    this->videoSource->close();
  }

  kv::algo::video_input::set_nested_algo_configuration(
    "video_reader", config, this->videoSource);

  auto lgcs = sfmConstraints->get_local_geo_cs();
  videoImporter.setData(config, stdString(videoPath), lgcs);

  try
  {
    if (this->videoSource)
    {
      this->videoSource->open(stdString(videoPath));
    }

    foreach (auto const& tool, this->tools)
    {
      tool->setEnabled(false);
    }

    videoImporter.start();
  }
  catch (kv::file_not_found_exception const& e)
  {
    qWarning() << e.what();
    this->videoSource->close();
    this->videoSource.reset();
  }
}

//-----------------------------------------------------------------------------
void MainWindowPrivate::addMaskSource(
  kv::config_block_sptr const& config, QString const& maskPath)
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
  kv::camera_perspective_sptr const& camera, int id)
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
  std::shared_ptr<kv::metadata_map::map_metadata_t> mdMap)
{
  this->videoMetadataMap = std::make_shared<kv::simple_metadata_map>(*mdMap);

  sfmConstraints->set_metadata(videoMetadataMap);



  this->UI.metadata->updateMetadata(mdMap);

  int num_cams_loaded_from_krtd = 0;

  if (this->project &&
      this->project->config->has_value("output_krtd_dir"))
  {
    qWarning() << "Loading project cameras with frames.count = "
               << this->frames.count();
    for (auto const& frame : this->frames)
    {
      auto frameName = qtString(this->getFrameName(frame.id)) + ".krtd";

      try
      {
        auto const& camera = kv::read_krtd_file(
          kvPath(frameName), kvPath(this->project->cameraPath));

        // Add camera to scene
        if (this->updateCamera(frame.id, camera))
        {
          ++num_cams_loaded_from_krtd;
        }
      }
      catch (...)
      {
        qWarning() << "failed to read camera file " << frameName
                   << " from " << this->project->cameraPath;
      }
    }
    this->UI.worldView->setCameras(this->cameraMap());
  }

  if(num_cams_loaded_from_krtd == 0)
  {
#define GET_K_CONFIG(type, name) \
  this->freestandingConfig->get_value<type>(bc + #name, K_def.name())

    kv::simple_camera_intrinsics K_def;
    const std::string bc = "video_reader:base_camera:";
    auto K = std::make_shared<kv::simple_camera_intrinsics>(
      GET_K_CONFIG(double, focal_length),
      GET_K_CONFIG(kv::vector_2d, principal_point),
      GET_K_CONFIG(double, aspect_ratio),
      GET_K_CONFIG(double, skew));

    auto baseCamera = kv::simple_camera_perspective();
    baseCamera.set_intrinsics(K);

    auto md = videoMetadataMap->metadata();
    kv::camera_map::map_camera_t camMap;
    if (!md.empty())
    {
      std::map<kv::frame_id_t, kv::metadata_sptr> mdMap;

      for (auto const& mdIter : md)
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
          kv::set_intrinsics_from_metadata(baseCamera, mdMap, im);
        }

        kv::local_geo_cs lgcs = sfmConstraints->get_local_geo_cs();
        camMap = kv::initialize_cameras_with_metadata(
          mdMap, baseCamera, lgcs);

        sfmConstraints->set_local_geo_cs(lgcs);

        if (this->project &&
            !sfmConstraints->get_local_geo_cs().origin().is_empty() &&
            !project->geoOriginFile.isEmpty())
        {
          saveGeoOrigin(project->geoOriginFile);
        }

      }
    }

    this->updateCameras(std::make_shared<kv::simple_camera_map>(camMap));
  }

  //find depth map paths
  if (this->project &&
      this->project->config->has_value("output_depth_dir"))
  {
    foreach (auto& frame, this->frames)
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

  this->UI.worldView->queueResetView();

  this->UI.worldView->initFrameSampling(this->frames.size());

  if (this->project)
  {
    for (auto const& tool : this->tools)
    {
      tool->setEnabled(true);
    }
  }
}

//-----------------------------------------------------------------------------
kv::camera_map_sptr MainWindowPrivate::cameraMap() const
{
  kv::camera_map::map_camera_t map;

  for (auto const& cd : this->frames)
  {
    if (cd.camera)
    {
      map.emplace(static_cast<kv::frame_id_t>(cd.id), cd.camera->GetCamera());
    }
  }

  return std::make_shared<kv::simple_camera_map>(map);
}

//-----------------------------------------------------------------------------
std::shared_ptr<std::map<kwiver::vital::frame_id_t, std::string>>
MainWindowPrivate::depthLookup() const
{
  auto lookup =
    std::make_shared<std::map<kwiver::vital::frame_id_t, std::string>>();

  for (auto cd : this->frames)
  {
    if (!cd.depthMapPath.isEmpty())
    {
      lookup->emplace(static_cast<kwiver::vital::frame_id_t>(cd.id),
                      qPrintable(cd.depthMapPath));
    }
  }

  return lookup;
}

//-----------------------------------------------------------------------------
void MainWindowPrivate::updateCameras(
  kv::camera_map_sptr const& cameras)
{
  auto allowExport = false;

  std::set<kv::frame_id_t> updated_frame_ids;
  foreach (auto const& iter, cameras->cameras())
  {
    auto cam_ptr =
      std::dynamic_pointer_cast<kv::camera_perspective>(iter.second);
    if (updateCamera(iter.first, cam_ptr))
    {
      updated_frame_ids.insert(iter.first);
      allowExport = allowExport || iter.second;
    }
  }

  for (auto& f : this->frames)
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
  this->UI.worldView->setCameras(cameras);

  this->UI.actionExportCameras->setEnabled(allowExport);
}

//-----------------------------------------------------------------------------
bool MainWindowPrivate::updateCamera(kv::frame_id_t frame,
                                     kv::camera_perspective_sptr cam)
{
  if (!cam)
  {
    return false;
  }

  auto* const fr = qtGet(this->frames, frame);
  if (!fr)
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
  // collect the set of frames to select from
  // TODO compute this set of frames only when it changes,
  // not every time the active camera changes.
  auto all_frames = this->frames.keys();
  std::set<kwiver::vital::frame_id_t> select_frames(all_frames.begin(),
                                                    all_frames.end());
  if (this->UI.actionTrackedFramesOnly->isChecked())
  {
    select_frames = tracks->all_frame_ids();
  }
  if (this->UI.actionKeyframesOnly->isChecked())
  {
    select_frames = tracks->keyframes();
  }

  bool next_frame_found = false;
  auto itr = select_frames.lower_bound(id);
  if (itr != select_frames.end())
  {
    // if stepping backwards we need to go back one iterator step
    // if we found the a frame greater than the one requested
    if (id < this->activeCameraIndex &&
        *itr >= this->activeCameraIndex &&
        itr != select_frames.begin())
    {
      --itr;
    }
    next_frame_found = true;
    id = *itr;
  }
  if (!next_frame_found)
  {
    // handle video playback
    if (this->UI.actionSlideshowPlay->isChecked())
    {
      if (this->UI.actionSlideshowLoop->isChecked())
      {
        this->UI.camera->setValue(*select_frames.begin());
        this->UI.cameraSpin->setValue(*select_frames.begin());
        return;
      }
      else
      {
        this->UI.actionSlideshowPlay->setChecked(false);
      }
    }
    // Set the active camera back to what it was.
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
  if (id >= 0 && id == this->activeDepthFrame)
  {
    this->resetActiveDepthMap(id);
    this->currentDepthFrame = id;
  }
  else // load from file
  {
    if (auto* const fr = qtGet(qAsConst(this->frames), id))
    {
      if (!fr->depthMapPath.isEmpty())
      {
        this->loadDepthMap(fr->depthMapPath);
        this->currentDepthFrame = id;
      }
    }
  }

  UI.worldView->setVolumeCurrentFrame(id);
}

//-----------------------------------------------------------------------------
void MainWindowPrivate::updateCameraView()
{
  if (this->activeCameraIndex < 1)
  {
    this->loadEmptyImage(0);
    this->UI.cameraView->setActiveFrame(static_cast<unsigned>(-1));
    this->UI.cameraView->clearLandmarks();
    this->UI.cameraView->clearGroundControlPoints();
    return;
  }

  this->UI.cameraView->setActiveFrame(
    static_cast<unsigned>(this->activeCameraIndex));

  auto* const activeFrame =
    qtGet(qAsConst(this->frames), this->activeCameraIndex);

  if (!activeFrame)
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
    this->UI.cameraView->clearGroundControlPoints();
    return;
  }

  // Show landmarks
  QHash<kv::track_id_t, kv::vector_2d> landmarkPoints;
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
        landmarkPoints.insert(id, kv::vector_2d(pp[0], pp[1]));
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
      if (state == track->end())
      {
        continue;
      }
      auto fts = std::dynamic_pointer_cast<kv::feature_track_state>(*state);
      if (fts && fts->feature)
      {
        auto const id = track->id();
        if (landmarkPoints.contains(id))
        {
          auto const& fp = fts->feature->loc();
          auto const& lp = landmarkPoints[id];
          this->UI.cameraView->addResidual(id, fp[0], fp[1], lp[0], lp[1], fts->inlier);
        }
      }
    }
  }

  this->groundControlPointsHelper->updateCameraViewPoints();
  this->rulerHelper->updateCameraViewRuler();
  this->UI.cameraView->render();
}

//-----------------------------------------------------------------------------
std::string MainWindowPrivate::getFrameName(kv::frame_id_t frameId)
{
  kwiver::vital::metadata_map::map_metadata_t md;
  if (videoMetadataMap)
  {
    md = videoMetadataMap->metadata();
  }
  return frameName(frameId, md);
}

//-----------------------------------------------------------------------------
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

    auto frameImg = videoSource->frame_image();
    if (!frameImg)
    {
      qWarning() << "Failed to read image for frame " << frame.id;
      this->loadEmptyImage(frame.camera);
      return;
    }
    auto sourceImg = frameImg->get_image();
    auto imageData = vitalToVtkImage(sourceImg);
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

      sfmConstraints->store_image_size(frame.id, dimensions[0], dimensions[1]);

      // Set frame name in camera view
      this->UI.cameraView->setImagePath(
        qtString(this->getFrameName(frame.id)));

      // Set image on views
      auto const size = QSize(dimensions[0], dimensions[1]);
      this->UI.cameraView->setImageData(imageData, size);
      this->UI.worldView->setImageData(imageData, size);

      // Update metadata view
      if (!this->videoMetadataMap)
      {
        this->UI.metadata->updateMetadata(kv::metadata_vector{});
      }
      else
      {
        auto md_map = this->videoMetadataMap->metadata();
        auto mdi = md_map.find(frame.id);
        if (mdi == md_map.end())
        {
          this->UI.metadata->updateMetadata(kwiver::vital::metadata_vector{});
        }
        else
        {
          this->UI.metadata->updateMetadata(mdi->second);
        }
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

  auto const ci = this->activeCameraIndex;
  if (auto* const activeFrame = qtGet(qAsConst(this->frames), ci))
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
  QObject::disconnect(this->UI.actionCancelComputation, nullptr,
                      this->activeTool, nullptr);

  // Update current tool
  this->activeTool = tool;

  // Connect actions
  if (tool)
  {
    QObject::connect(this->UI.actionCancelComputation, &QAction::triggered,
                     tool, &AbstractTool::cancel);
    QObject::connect(this->UI.actionCancelComputation, &QAction::triggered,
                     project.data(), &Project::write);
    QObject::connect(this->UI.actionQuit, &QAction::triggered,
                     tool, &AbstractTool::cancel);
    QObject::connect(tool, &AbstractTool::completed,
                     project.data(), &Project::write);
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
    if (value < 100)
    {
      taskId = this->UI.progressWidget->addTask(desc, 0, 0, 0);
      this->progressIds.insert(object, taskId);
    }
    return;
  }
  else
  {
    taskId = this->progressIds.value(object);
  }

  this->UI.progressWidget->setTaskText(taskId, desc);
  this->UI.progressWidget->setProgressValue(taskId, value);
  switch (value)
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

//-----------------------------------------------------------------------------
std::string MainWindowPrivate::roiToString()
{
  double minpt[3], maxpt[3];
  this->roi->GetXMin(minpt);
  this->roi->GetXMax(maxpt);
  std::ostringstream stream;
  stream << minpt[0] << " " << minpt[1] << " " << minpt[2] << " "
         << maxpt[0] << " " << maxpt[1] << " " << maxpt[2];
  return stream.str();
}

//-----------------------------------------------------------------------------
void MainWindowPrivate::loadroi(const std::string& roistr)
{
  double minpt[3], maxpt[3];
  std::istringstream stream(roistr);
  stream >> minpt[0] >> minpt[1] >> minpt[2]
         >> maxpt[0] >> maxpt[1] >> maxpt[2];
  this->roi->SetXMin(minpt);
  this->roi->SetXMax(maxpt);
  UI.worldView->setROI(roi.GetPointer(), true);
}

//-----------------------------------------------------------------------------
void MainWindowPrivate::resetActiveDepthMap(int frame)
{
  this->depthReader->SetFileName("");
  this->depthFilter->RemoveAllInputConnections(0);
  this->depthFilter->SetInputData(this->activeDepth);

  this->UI.depthMapView->setValidDepthInput(true);
  this->UI.worldView->setValidDepthInput(true);

  if (auto* const fr = qtGet(qAsConst(this->frames), frame))
  {
    this->depthFilter->SetCamera(fr->camera);
  }
  this->UI.worldView->updateDepthMap();
  this->UI.depthMapView->updateView(true);
  this->UI.actionExportDepthPoints->setEnabled(true);

  this->currentDepthFrame = frame;
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
  d->addTool(new InitCamerasLandmarksTool(this), this);
  d->addTool(new SaveFrameTool(this), this);
  d->addTool(new ComputeAllDepthTool(this), this);
  d->addTool(new FuseDepthTool(this), this);

  d->toolMenu = d->UI.menuAdvanced;
  d->toolSeparator =
    d->UI.menuAdvanced->addSeparator();
  d->addTool(new TrackFilterTool(this), this);
  d->addTool(new TriangulateTool(this), this);
  d->addTool(new BundleAdjustTool(this), this);
  d->addTool(new NeckerReversalTool(this), this);
  d->addTool(new CanonicalTransformTool(this), this);
  d->addTool(new SaveKeyFrameTool(this), this);
  d->addTool(new ComputeDepthTool(this), this);

  d->UI.menuView->addSeparator();
  d->UI.menuView->addAction(d->UI.cameraViewDock->toggleViewAction());
  d->UI.menuView->addAction(d->UI.cameraSelectorDock->toggleViewAction());
  d->UI.menuView->addAction(d->UI.metadataDock->toggleViewAction());
  d->UI.menuView->addAction(d->UI.groundControlPointsDock->toggleViewAction());
  d->UI.menuView->addAction(d->UI.depthMapViewDock->toggleViewAction());

  d->UI.playSlideshowButton->setDefaultAction(d->UI.actionSlideshowPlay);
  d->UI.loopSlideshowButton->setDefaultAction(d->UI.actionSlideshowLoop);

  connect(d->UI.actionQuit, &QAction::triggered,
          qApp, &QCoreApplication::quit);

  connect(d->UI.actionNewProject, &QAction::triggered,
          this, &MainWindow::newProject);
  connect(d->UI.actionOpenProject, &QAction::triggered,
          this, &MainWindow::openProject);
  connect(d->UI.actionImportImagery, &QAction::triggered,
          this, &MainWindow::openImagery);
  connect(d->UI.actionImportMasks, &QAction::triggered,
          this, &MainWindow::openMaskImagery);
  connect(d->UI.actionImportCameras, &QAction::triggered,
          this, &MainWindow::openCameras);
  connect(d->UI.actionImportTracks, &QAction::triggered,
          this, &MainWindow::openTracks);
  connect(d->UI.actionImportLandmarks, &QAction::triggered,
          this, &MainWindow::openLandmarks);
  connect(d->UI.actionImportGroundControlPoints, &QAction::triggered,
          this, &MainWindow::openGroundControlPoints);

  connect(d->UI.actionShowWorldAxes, &QAction::toggled,
          d->UI.worldView, &WorldView::setAxesVisible);

  connect(d->UI.actionExportCameras, &QAction::triggered,
          this, QOverload<>::of(&MainWindow::saveCameras));
  connect(d->UI.actionExportLandmarks, &QAction::triggered,
          this, QOverload<>::of(&MainWindow::saveLandmarks));
  connect(d->UI.actionExportGroundControlPoints, &QAction::triggered,
          this, QOverload<>::of(&MainWindow::saveGroundControlPoints));
  connect(d->UI.actionExportVolume, &QAction::triggered,
          this, &MainWindow::saveVolume);
  connect(d->UI.actionExportFusedMesh, &QAction::triggered,
          this, &MainWindow::saveFusedMesh);
  connect(d->UI.actionExportDepthPoints, &QAction::triggered,
          this, QOverload<>::of(&MainWindow::saveDepthPoints));
  connect(d->UI.actionExportTracks, &QAction::triggered,
          this, QOverload<>::of(&MainWindow::saveTracks));

  connect(d->UI.worldView, &WorldView::depthMapEnabled,
          this, &MainWindow::enableSaveDepthPoints);

  connect(d->UI.actionShowMatchMatrix, &QAction::triggered,
          this, &MainWindow::showMatchMatrix);

  connect(&d->toolDispatcher, QOverload<QObject*>::of(&QSignalMapper::mapped),
          this, &MainWindow::executeTool);

  connect(d->UI.actionSetBackgroundColor, &QAction::triggered,
          this, &MainWindow::setViewBackroundColor);

  connect(d->UI.actionAbout, &QAction::triggered,
          this, &MainWindow::showAboutDialog);
  connect(d->UI.actionShowManual, &QAction::triggered,
          this, &MainWindow::showUserManual);

  connect(&d->slideTimer, &QTimer::timeout, this, &MainWindow::nextSlide);
  connect(d->UI.actionSlideshowPlay, &QAction::toggled,
          this, &MainWindow::setSlideshowPlaying);
  connect(d->UI.slideSpeed, &QAbstractSlider::valueChanged,
          this, &MainWindow::setSlideSpeed);

  connect(d->UI.camera, &QAbstractSlider::valueChanged,
          this, &MainWindow::setActiveCamera);

  connect(d->UI.worldView, &WorldView::fusedMeshEnabled,
          this, &MainWindow::enableSaveFusedMesh);

  connect(d->UI.worldView, &WorldView::depthMapThresholdsChanged,
          d->UI.depthMapView, &DepthMapView::updateThresholds);

  connect(d->UI.depthMapViewDock, &QDockWidget::visibilityChanged,
          d->UI.depthMapView, &DepthMapView::updateView);

  this->setSlideSpeed(d->UI.slideSpeed->value());

#ifdef VTKWEBGLEXPORTER
  d->UI.actionWebGLScene->setVisible(true);
  connect(d->UI.actionWebGLScene, &QAction::triggered,
          this, &MainWindow::saveWebGLScene);
#endif

  // Set up UI persistence and restore previous state
  auto const sdItem = new qtUiState::Item<int, QSlider>(
    d->UI.slideSpeed, &QSlider::value, &QSlider::setValue);
  d->uiState.map("SlideSpeed", sdItem);

  d->viewBackgroundColor = new StateValue<QColor>{Qt::black},
  d->uiState.map("ViewBackground", d->viewBackgroundColor);

  d->uiState.mapChecked("Antialiasing", d->UI.actionAntialiasing);

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
  d->UI.worldView->setDepthGeometryFilter(d->depthGeometryFilter);
  d->UI.depthMapView->setDepthGeometryFilter(d->depthGeometryFilter);

  d->UI.worldView->resetView();

  // Set up the progress widget
  d->UI.progressWidget->setAutoHide(true);

  // Ground control points
  d->groundControlPointsHelper = new GroundControlPointsHelper(this);
  connect(d->groundControlPointsHelper,
          &GroundControlPointsHelper::pointCountChanged,
          this, [d](size_t count) {
            d->UI.actionExportGroundControlPoints->setEnabled(count > 0);
          });
  connect(d->UI.worldView, &WorldView::pointPlacementEnabled,
          d->groundControlPointsHelper,
          &GroundControlPointsHelper::enableWidgets);
  d->UI.groundControlPoints->setHelper(d->groundControlPointsHelper);

  // Ruler widget
  d->rulerHelper = new RulerHelper(this);
  connect(d->UI.worldView, &WorldView::rulerEnabled,
          d->rulerHelper, &RulerHelper::enableWidgets);
  connect(d->UI.worldView, &WorldView::rulerReset,
          d->rulerHelper, &RulerHelper::resetRuler);

  // Antialiasing
  connect(d->UI.actionAntialiasing, &QAction::toggled,
          this, &MainWindow::enableAntiAliasing);
  this->enableAntiAliasing(d->UI.actionAntialiasing->isChecked());

  // Common ROI
  // Unitil the bounding box is initialized, the bounds are going to be
  // [VTK_DOUBLE_MIN, VTK_DOUBLE_MAX] in all directions.
  d->UI.worldView->setROI(d->roi);
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
                                             "Video files (" +
      videoFilters + ");;"
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
                                             "Video files (" +
      videoFilters + ");;"
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
void MainWindow::openGroundControlPoints()
{
  auto const paths = QFileDialog::getOpenFileNames(
    this, "Open Ground Control Points", QString(),
    "GeoJSON Files (*.json);;"
    "All Files (*)");

  for (auto const& path : paths)
  {
    this->loadGroundControlPoints(path);
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
    d->project->config->set_value(
      "output_krtd_dir",
      kvPath(d->project->getContingentRelativePath(d->project->cameraPath)));

    if (!d->sfmConstraints->get_local_geo_cs().origin().is_empty() &&
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
    qWarning() << "Unable to set current working directory "
                  "to project directory"
               << d->project->workingDir.absolutePath();
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
    d->UI.worldView->loadVolume(d->project->volumePath);
  }

  if (d->project->config->has_value("geo_origin_file"))
  {
    if (QFileInfo{d->project->geoOriginFile}.isFile())
    {
      kv::local_geo_cs lgcs;
      kv::read_local_geo_cs_from_file(
        lgcs, stdString(d->project->geoOriginFile));

      d->sfmConstraints->set_local_geo_cs(lgcs);
    }
    else
    {
      qWarning() << "Failed to open geo origin file "
                 << d->project->geoOriginFile << ". File does not exist.";
    }
  }

  if (d->project->config->has_value("ROI"))
  {
    d->loadroi(d->project->ROI);
  }

  d->UI.worldView->queueResetView();

  foreach (auto const& tool, d->tools)
  {
    tool->setEnabled(true);
  }

  d->setActiveCamera(d->activeCameraIndex);

  // Load ground control points after cameras are loaded
  if (d->project->config->has_value("ground_control_points_file"))
  {
    this->loadGroundControlPoints(d->project->groundControlPath);
  }

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
    d->project->config->set_value(
      "output_krtd_dir",
      kvPath(d->project->getContingentRelativePath(d->project->cameraPath)));

    if (!d->sfmConstraints->get_local_geo_cs().origin().is_empty() &&
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
    auto const& camera = kv::read_krtd_file(kvPath(path));
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

  namespace kac = kwiver::arrows::core;

  try
  {
    using tsi_uptr = std::unique_ptr<kv::track_set_implementation>;

    auto tracks = kv::read_feature_track_file(kvPath(path));
    if (tracks)
    {
      // check for older zero-based track files
      if (tracks->first_frame() == 0)
      {
        qWarning() << "Loaded tracks have zero-based indexing, "
                      "shifting to one-based indexing";
        // shift tracks to start with frame one
        std::vector<kv::track_sptr> new_tracks;
        for (auto const& track : tracks->tracks())
        {
          auto new_track = kv::track::create(track->data());
          new_track->set_id(track->id());
          for (auto const& ts : *track)
          {
            auto fts = std::dynamic_pointer_cast<kv::feature_track_state>(ts);
            auto new_fts = std::make_shared<kv::feature_track_state>(
              ts->frame() + 1, fts->feature, fts->descriptor);
            new_track->append(new_fts);
          }
          new_tracks.push_back(new_track);
        }

        auto tks_temp =
          std::make_shared<kv::feature_track_set>(
            tsi_uptr{new kac::frame_index_track_set_impl{new_tracks}});
        tks_temp->set_frame_data(tracks->all_frame_data());
        tracks = tks_temp;
      }
      else
      {
        auto tks_temp = std::make_shared<kv::feature_track_set>(
          tsi_uptr{new kac::frame_index_track_set_impl{tracks->tracks()}});
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
      d->UI.actionTrackedFramesOnly->setEnabled(!tracks->tracks().empty());
    }
  }
  catch (std::exception const& e)
  {
    qWarning() << "failed to read tracks from" << path
               << " with error: " << e.what();
  }
}

//-----------------------------------------------------------------------------
void MainWindow::loadLandmarks(QString const& path)
{
  QTE_D();

  try
  {
    auto const& landmarks = kv::read_ply_file(kvPath(path));
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
void MainWindow::loadGroundControlPoints(QString const& path)
{
  QTE_D();

  if (d->groundControlPointsHelper->readGroundControlPoints(path))
  {
    d->UI.actionExportGroundControlPoints->setEnabled(
      d->groundControlPointsHelper->groundControlPoints().size());
  }
}

//-----------------------------------------------------------------------------
void MainWindow::saveLandmarks()
{
  QTE_D();

  auto const name = d->project->workingDir.dirName();
  auto const path = QFileDialog::getSaveFileName(
    this, "Export Landmarks", name + QString("_landmarks.ply"),
    "Landmark file (*.ply);;"
    "LAS file (*.las);;"
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
    if (QFileInfo(path).suffix() == "las")
    {
      auto lgcs = d->sfmConstraints->get_local_geo_cs();
      kwiver::maptk::write_pdal(stdString(path), lgcs, d->landmarks);
    }
    else
    {
      kv::write_ply_file(d->landmarks, kvPath(path));

      if (writeToProject && d->project)
      {
        d->project->config->set_value(
          "output_ply_file",
          kvPath(d->project->getContingentRelativePath(path)));
      }
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
void MainWindow::saveGroundControlPoints()
{
  QTE_D();

  auto const name = d->project->workingDir.dirName();
  auto const path = QFileDialog::getSaveFileName(
    this, "Export Ground Control Points", name + QString("_gcps.json"),
    "GeoJSON file (*.json);;"
    "All Files (*)");

  if (!path.isEmpty())
  {
    this->saveGroundControlPoints(path, true);
  }
}

//-----------------------------------------------------------------------------
void MainWindow::saveGroundControlPoints(
  QString const& path, bool writeToProject)
{
  QTE_D();

  if (d->groundControlPointsHelper->writeGroundControlPoints(path, this))
  {
    try
    {
      if (writeToProject && d->project)
      {
        d->project->groundControlPath =
          d->project->getContingentRelativePath(path);
        d->project->config->set_value(
          "ground_control_points_file",
          kvPath(d->project->groundControlPath));
        d->project->write();
      }
    }
    catch (...)
    {
      auto const msg =
        QStringLiteral("An error occurred while exporting "
                       "ground control points to \"%1\". "
                       "The output file may not have been written correctly.");
      QMessageBox::critical(this, QStringLiteral("Export error"),
                            msg.arg(d->project->groundControlPath));
    }
  }
}

//-----------------------------------------------------------------------------
void MainWindow::saveTracks()
{
  QTE_D();

  auto const name = d->project->workingDir.dirName();
  auto const path = QFileDialog::getSaveFileName(
    this, "Export Tracks", name + QString("_tracks.txt"),
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
    kv::write_feature_track_file(d->tracks, kvPath(path));

    if (writeToProject && d->project)
    {
      d->project->config->set_value(
        "output_tracks_file",
        kvPath(d->project->getContingentRelativePath(path)));
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

  auto out = QHash<QString, kv::camera_perspective_sptr>();
  auto willOverwrite = QStringList();

  auto qdir = QDir(path);
  auto entry_info_list = qdir.entryInfoList();
  const QString cam_extension = "krtd";

  for (auto &ent : entry_info_list)
  {
    if (ent.isFile() && ent.suffix() == cam_extension)
    {
      auto del_file_str = ent.absoluteFilePath();
      QFile f(del_file_str);
      f.remove();
    }
  }

  for (auto const& cd : d->frames)
  {
    if (cd.camera)
    {
      auto const camera = cd.camera->GetCamera();
      if (camera)
      {
        auto cameraName = qtString(d->getFrameName(cd.id) + "."
                                   + stdString(cam_extension));
        auto const filepath = QDir{path}.filePath(cameraName);
        out.insert(filepath, camera);

        if (QFileInfo::exists(filepath))
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
                   "Do you wish to continue?",
                   QMessageBox::Cancel, this);

    auto* const myOverwrite =
      mb.addButton("&Overwrite", QMessageBox::AcceptRole);
    mb.setDetailedText("The following file(s) will be overwritten:\n  " +
                       willOverwrite.join("\n  "));

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
        std::dynamic_pointer_cast<kv::camera_perspective>(iter.value());
      kv::write_krtd_file(*cam_ptr, kvPath(iter.key()));
    }
    catch (...)
    {
      errors.append(iter.key());
    }
  }

  if (writeToProject && d->project)
  {
    d->project->config->set_value(
      "output_krtd_dir",
      kvPath(d->project->getContingentRelativePath(path)));
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

  d->project->config->set_value("output_depth_dir", kvPath(
    d->project->getContingentRelativePath(d->project->depthPath)));

  d->project->config->set_value("ROI", d->roiToString());
  vtkNew<vtkXMLImageDataWriter> writerI;
  auto const filepath = QDir{path}.filePath(filename);
  writerI->SetFileName(qPrintable(filepath));
  writerI->AddInputDataObject(d->activeDepth.Get());
  writerI->SetDataModeToBinary();
  writerI->Write();

  if (auto* const activeFrame = qtGet(d->frames, d->activeDepthFrame))
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
  QTE_D();

  QString name;
  if (d->currentDepthFrame > 0)
  {
    name = qtString(d->getFrameName(d->currentDepthFrame) + "_depth.ply");
  }
  auto const path = QFileDialog::getSaveFileName(
    this, "Export Depth Point Cloud", name,
    "PLY file (*.ply);;"
    "LAS file (*.las);;"
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
    auto lgcs = d->sfmConstraints->get_local_geo_cs();
    d->UI.worldView->saveDepthPoints(path, lgcs);
    d->project->config->set_value(
      "depthmaps_images_file",
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

//-----------------------------------------------------------------------------
void MainWindow::saveGeoOrigin(QString const& path)
{
  QTE_D();

  d->saveGeoOrigin(path);
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
void MainWindow::enableSaveFusedMesh(bool state)
{
  QTE_D();

  d->UI.actionExportVolume->setEnabled(state);
  d->UI.actionExportFusedMesh->setEnabled(state);
}

//-----------------------------------------------------------------------------
void MainWindow::saveVolume()
{
  QTE_D();

  auto const name = d->project->workingDir.dirName();
  auto const path = QFileDialog::getSaveFileName(
    this, "Export Volume", name + QString("_volume.vts"),
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

  d->project->config->set_value("ROI", d->roiToString());
}

//-----------------------------------------------------------------------------
void MainWindow::saveFusedMesh()
{
  QTE_D();

  auto const name = d->project->workingDir.dirName();
  auto const path = QFileDialog::getSaveFileName(
    this, "Export Fused Mesh", name + QString("_fused_mesh.ply"),
    "PLY File (*.ply);;"
    "LAS File (*.las);;"
    "VTK Polydata (*.vtp);;"
    "All Files (*)");

  try
  {
    if (!path.isEmpty())
    {
      auto lgcs = d->sfmConstraints->get_local_geo_cs();
      d->UI.worldView->saveFusedMesh(path, lgcs);
    }
  }
  catch (...)
  {
    auto const msg =
      QString("An error occurred while exporting the mesh to \"%1\". "
        "The output file may not have been written correctly.");
    QMessageBox::critical(this, "Export error", msg.arg(path));
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
    auto const de = static_cast<double>(speed) * 0.1;
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

  auto const tool = qobject_cast<AbstractTool*>(object);
  if (tool && !d->activeTool)
  {
    // try to reset the ROI if invalid
    kv::vector_3d min_pt, max_pt;
    d->roi->GetXMin(min_pt.data());
    d->roi->GetXMax(max_pt.data());
    if (((max_pt - min_pt).array() <= 0.0).any())
    {
      d->UI.worldView->resetROI();
      d->project->config->set_value("ROI", d->roiToString());
    }

    d->setActiveTool(tool);
    tool->setActiveFrame(d->activeCameraIndex);
    tool->setTracks(d->tracks);
    tool->setCameras(d->cameraMap());
    tool->setLandmarks(d->landmarks);
    tool->setSfmConstraints(d->sfmConstraints);
    tool->setVideoPath(stdString(d->videoPath));
    tool->setMaskPath(stdString(d->maskPath));
    tool->setConfig(d->project->config);
    tool->setROI(d->roi.Get());
    tool->setDepthLookup(d->depthLookup());
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

//-----------------------------------------------------------------------------
void MainWindow::reportToolError(QString const& msg)
{
    QMessageBox::critical(this, "Error in Tool",
                          "Tool execution failed: " + msg);
}

//-----------------------------------------------------------------------------
void MainWindow::acceptToolInterimResults(std::shared_ptr<ToolData> data)
{
  this->acceptToolResults(data, false);
}

//-----------------------------------------------------------------------------
void MainWindow::acceptToolFinalResults()
{
  QTE_D();

  if (d->activeTool)
  {
    this->acceptToolResults(d->activeTool->data(), true);
    this->saveToolResults();
    // Signal tool execution as complete to the progress widget
    d->updateProgress(d->activeTool,
                      d->activeTool->description(),
                      100);
  }
  d->setActiveTool(0);
}

//-----------------------------------------------------------------------------
void MainWindow::acceptToolResults(
  std::shared_ptr<ToolData> data, bool isFinal)
{
  QTE_D();
  // if all the update variables are Null then trigger a GUI update after
  // extracting the data otherwise we've already triggered an update that
  // hasn't happened yet, so don't trigger another
  bool updateNeeded = !d->toolUpdateCameras &&
                      !d->toolUpdateLandmarks &&
                      !d->toolUpdateTracks &&
                      !d->toolUpdateTrackChanges &&
                      !d->toolUpdateDepth &&
                      !d->toolUpdateVolume &&
                      d->toolUpdateActiveFrame < 0;

  if (d->activeTool)
  {
    auto const outputs = d->activeTool->outputs();

    d->toolUpdateCameras = NULL;
    d->toolUpdateLandmarks = NULL;
    d->toolUpdateTracks = NULL;
    d->toolUpdateTrackChanges = NULL;
    d->toolUpdateActiveFrame = -1;
    d->toolUpdateDepth = NULL;
    d->toolUpdateVolume = NULL;
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
    if (outputs.testFlag(AbstractTool::TrackChanges))
    {
      d->toolUpdateTrackChanges = data->track_changes;
    }
    if (outputs.testFlag(AbstractTool::Depth))
    {
      d->toolUpdateDepth = data->active_depth;
    }
    if (outputs.testFlag(AbstractTool::ActiveFrame))
    {
      d->toolUpdateActiveFrame = static_cast<int>(data->activeFrame);
    }
    if (outputs.testFlag(AbstractTool::BatchDepth))
    {
      d->toolSaveDepthFlag = true;
    }
    if (outputs.testFlag(AbstractTool::Fusion))
    {
      d->toolUpdateVolume = data->volume;
    }
    // Update tool progress
    d->updateProgress(d->activeTool,
                      d->activeTool->description(),
                      d->activeTool->progress());
  }

  if (isFinal)
  {
    bool update_origin = d->toolUpdateLandmarks != nullptr;

    // Force immediate update on tool finish so we ensure update before saving
    updateToolResults();

    // If the landmarks changed, then update the geo coordinate system
    if (update_origin)
    {
      // if a local geo coordinate system exists,
      // recompute the origin to center the points
      if (!d->sfmConstraints->get_local_geo_cs().origin().is_empty())
      {
        auto offset = d->centerLandmarks();
        d->shiftGeoOrigin(offset);
      }
    }
  }
  else if (updateNeeded)
  {
    QTimer::singleShot(1000, this, &MainWindow::updateToolResults);
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

    if (!d->sfmConstraints->get_local_geo_cs().origin().is_empty() &&
        !d->project->geoOriginFile.isEmpty())
    {
      saveGeoOrigin(d->project->geoOriginFile);
    }

    if (!outputs.testFlag(AbstractTool::BatchDepth) &&
        outputs.testFlag(AbstractTool::Depth))
    {
      saveDepthImage(d->project->depthPath);
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
    d->UI.actionTrackedFramesOnly->setEnabled(!d->tracks->tracks().empty());
    d->toolUpdateTracks = NULL;
  }
  if (d->toolUpdateTrackChanges)
  {
    //SET THE TRACK FLAGS HERE
    if (d->tracks)
    {
      for (auto const& change: d->toolUpdateTrackChanges->m_changes)
      {
        auto tk = d->tracks->get_track(change.track_id_);
        if (tk)
        {
          auto tsi = tk->find(change.frame_id_);
          if (tsi != tk->end())
          {
            auto fts = std::dynamic_pointer_cast<kwiver::vital::feature_track_state>(*tsi);
            if (fts)
            {
              fts->inlier = change.inlier_;
            }
          }
        }
      }
    }
    d->toolUpdateTrackChanges = NULL;
  }

  if (d->toolUpdateDepth)
  {
    d->activeDepth = d->toolUpdateDepth;
    d->activeDepthFrame = d->toolUpdateActiveFrame;
    d->currentDepthFrame = d->toolUpdateActiveFrame;

    // In batch depth, each update is a full depth map from a different ref
    // frame that must be saved
    if (d->toolSaveDepthFlag)
    {
      saveDepthImage(d->project->depthPath);
      d->toolSaveDepthFlag = false;
    }

    d->toolUpdateDepth = NULL;
  }
  if (d->toolUpdateVolume)
  {
    d->UI.worldView->setVolume(d->toolUpdateVolume);
    d->toolUpdateVolume = NULL;
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
  std::shared_ptr<kv::metadata_map::map_metadata_t> mdMap)
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
    auto frames = std::vector<kv::frame_id_t>();
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
void MainWindow::applySimilarityTransform()
{
  QTE_D();

  auto lgcs = d->sfmConstraints->get_local_geo_cs();
  if (lgcs.origin().is_empty())
  {
    qWarning() << "Can't apply similarity transform without local coordinate system";
    return;
  }

  auto local_crs = lgcs.origin().crs();

  std::vector<kwiver::vital::vector_3d> from_pts, to_pts;
  auto gcp_map = d->groundControlPointsHelper->groundControlPoints();
  for ( auto gcp : gcp_map )
  {
    if ( gcp.second->is_geo_loc_user_provided() )
    {
      from_pts.push_back( gcp.second->loc() );
      kwiver::vital::vector_3d to_pt;
      to_pt.block< 2, 1>( 0, 0 ) =
        gcp.second->geo_loc().location( local_crs ) - lgcs.origin().location();
      to_pt(2) = gcp.second->elevation() - lgcs.origin_altitude();
      to_pts.push_back( to_pt );
    }
  }

  // TODO: enforce by disabling button if there is not enough points
  if (from_pts.size() < 3)
  {
    QMessageBox::information(
      this, "",
      "Must have at least three user defined ground control points to"
      " apply similarity transform.");
    return;
  }

  // Merge project config with default config file
  auto const config = readConfig("gui_st_estimator.conf");

  // Check configuration
  if (!config)
  {
    QMessageBox::critical(
      this, "Configuration error",
      "No configuration data was found for similarity estimator. "
      "Please check your installation.");
    return;
  }

  config->merge_config(d->project->config);
  if (! kv::algo::estimate_similarity_transform::check_nested_algo_configuration("st_estimator", config) )
  {
    QMessageBox::critical(
      this, "Configuration error",
      "An error was found in the similarity estimator configuration.");
    return;
  }

  // Create the similarity transform from the ground control points
  kv::algo::estimate_similarity_transform_sptr st_estimator;
  kv::algo::estimate_similarity_transform::set_nested_algo_configuration(
      "st_estimator", config, st_estimator);

  // initialize identity transform
  kwiver::vital::similarity_d sim_transform;

  sim_transform = st_estimator->estimate_transform(from_pts, to_pts);

  // Transform landmarks
  d->landmarks = kwiver::arrows::transform(d->landmarks, sim_transform);

  // Transform cameras
  auto camera_map = d->cameraMap();
  camera_map = kwiver::arrows::transform(camera_map, sim_transform);
  d->updateCameras(camera_map);

  // Transform GCP's
  for ( auto gcp : gcp_map )
  {
    auto gcp_loc = gcp.second->loc();
    gcp.second->set_loc(sim_transform*gcp_loc);
  }

  // Transform ROI
  kwiver::vital::vector_3d minPt;
  kwiver::vital::vector_3d maxPt;
  d->roi->GetXMin(minPt.data());
  d->roi->GetXMax(maxPt.data());
  std::vector<kwiver::vital::vector_3d> boundPts = {minPt, maxPt};

  for (int i=0; i < 2; ++i)
  {
    for (int j=0; j < 2; ++j)
    {
      for (int k =0; k < 2; ++k)
      {
        kwiver::vital::vector_3d currPt(boundPts[i][0],
                                        boundPts[j][1],
                                        boundPts[k][3]);
        kwiver::vital::vector_3d newPt = sim_transform*currPt;
        for (int l = 0; l < 3; ++l)
        {
          if (newPt[l] < minPt[l])
          {
            minPt[l] = newPt[l];
          }
          if (newPt[l] > maxPt[l])
          {
            maxPt[l] = newPt[l];
          }
        }
      }
    }
  }

  d->roi->SetXMin(minPt.data());
  d->roi->SetXMax(maxPt.data());

  // Scale all the depth maps
  for (auto const& f : d->frames)
  {
    if (f.depthMapPath.size() > 0)
    {
      vtkNew<vtkXMLImageDataReader> imageReader;
      imageReader->SetFileName(qPrintable(f.depthMapPath));
      imageReader->Update();
      vtkSmartPointer<vtkImageData> depthImg = imageReader->GetOutput();

      vtkSmartPointer<vtkDoubleArray> depthData = vtkDoubleArray::FastDownCast(
          depthImg->GetPointData()->GetAbstractArray("Depths"));
      auto numValues = depthData->GetNumberOfValues();
      for (vtkIdType i = 0; i < numValues; ++i)
      {
        depthData->SetValue(i, sim_transform.scale()*depthData->GetValue(i));
      }

      // Replace active depth map if needed
      if (f.id == d->currentDepthFrame)
      {
        d->activeDepth = depthImg;
        d->resetActiveDepthMap(f.id);
      }

      vtkNew<vtkXMLImageDataWriter> imageWriter;
      imageWriter->SetFileName(qPrintable(f.depthMapPath));
      imageWriter->AddInputDataObject(depthImg.Get());
      imageWriter->SetDataModeToBinary();
      imageWriter->Write();
    }
  }

  // Invalidate the fusion volume
  d->UI.worldView->resetVolume();
}

//-----------------------------------------------------------------------------
void MainWindow::updateVideoImportProgress(QString const& desc, int progress)
{
  QTE_D();

  d->updateProgress(this->sender(), desc, progress);
}

//-----------------------------------------------------------------------------
WorldView* MainWindow::worldView()
{
  QTE_D();

  return d->UI.worldView;
}

//-----------------------------------------------------------------------------
CameraView* MainWindow::cameraView()
{
  QTE_D();

  return d->UI.cameraView;
}

//-----------------------------------------------------------------------------
kwiver::vital::local_geo_cs MainWindow::localGeoCoordinateSystem() const
{
  QTE_D();

  return d->sfmConstraints->get_local_geo_cs();
}

//-----------------------------------------------------------------------------
vtkMaptkCamera* MainWindow::activeCamera()
{
  QTE_D();

  if (d->activeCameraIndex < 1 || d->frames.empty())
  {
    return nullptr;
  }

  auto* const activeFrame = qtGet(d->frames, d->activeCameraIndex);
  return (activeFrame ? activeFrame->camera : nullptr);
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
