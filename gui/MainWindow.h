// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#ifndef TELESCULPTOR_MAINWINDOW_H_
#define TELESCULPTOR_MAINWINDOW_H_

#include <qtGlobal.h>

#include <QMainWindow>

#include <memory>

#include <vital/types/local_geo_cs.h>
#include <vital/types/metadata_map.h>

class CameraView;
class ToolData;
class WorldView;
namespace kwiver {
namespace arrows {
namespace vtk {
  class vtkKwiverCamera;
}}}

class MainWindowPrivate;

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(QWidget* parent = nullptr, Qt::WindowFlags flags = {});
  ~MainWindow() override;

  kwiver::vital::frame_id_t activeFrame() const;
  kwiver::arrows::vtk::vtkKwiverCamera* activeCamera() const;

  WorldView* worldView() const;
  CameraView* cameraView() const;

  QString frameName(kwiver::vital::frame_id_t) const;

  kwiver::vital::local_geo_cs localGeoCoordinateSystem() const;

public slots:
  void newProject();

  void openProject();
  void openImagery();
  void openMaskImagery();
  void openCameras();
  void openTracks();
  void openLandmarks();
  void openGroundControlPoints();
  void openMesh();

  void loadProject(QString const& path);
  void loadImagery(QString const& path);
  void loadMaskImagery(QString const& path);
  void loadImage(QString const& path);
  void loadVideo(QString const& path);
  void loadMaskImage(QString const& path);
  void loadMaskVideo(QString const& path);
  void loadCamera(QString const& path);
  void loadTracks(QString const& path);
  void loadLandmarks(QString const& path);
  void loadGroundControlPoints(QString const& path);
  void loadMesh(QString const& path);

  void saveCameras();
  void saveCameras(QString const& path, bool writeToProject = true);
  void saveLandmarks();
  void saveLandmarks(QString const& path, bool writeToProject = true);
  void saveGroundControlPoints();
  void saveGroundControlPoints(QString const& path, bool writeToProject = true);
  void saveTracks();
  void saveTracks(QString const& path, bool writeToProject = true);
  void saveDepthPoints();
  void saveDepthPoints(QString const& path);
  void saveDepthImage(QString const& path);
  void saveGeoOrigin(QString const& path);
  void saveToolResults();
  void acceptToolSaveResults(std::shared_ptr<ToolData> data);

  void applySimilarityTransform();
  void computeCamera();

  void saveWebGLScene();

  void saveVolume();
  void enableSaveVolume(bool);
  void enableSaveFusedMesh(bool);
  void saveFusedMesh();
  void saveFusedMeshFrameColors();

  void enableSaveDepthPoints(bool);

  void setActiveFrame(kwiver::vital::frame_id_t);

  void setViewBackroundColor();

  void showMatchMatrix();

  void showAboutDialog();
  void showUserManual();

  void updateToolProgress(QString const&, int);

protected slots:
  void setSlideSpeed(int);
  void setSlideshowPlaying(bool);
  void nextSlide();

  void executeTool(QObject*);
  void reportToolError(QString const&);
  void acceptToolInterimResults(std::shared_ptr<ToolData> data);
  void acceptToolFinalResults();
  void updateToolResults();
  void addFrame(int);
  void updateFrames(
    std::shared_ptr<kwiver::vital::metadata_map::map_metadata_t>);
  void enableAntiAliasing(bool enable);

  void setIgnoreMetadata(bool);
  void setVariableLens(bool);
  void setFixGeoOrigin(bool);

private:
  void acceptToolResults(std::shared_ptr<ToolData> data, bool isFinal);
  void setComputeOption(std::string const& name, bool state);

  QTE_DECLARE_PRIVATE_RPTR(MainWindow)
  QTE_DECLARE_PRIVATE(MainWindow)

  QTE_DISABLE_COPY(MainWindow)
};

#endif
