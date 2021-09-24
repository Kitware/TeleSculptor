// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#ifndef TELESCULPTOR_WORLDVIEW_H_
#define TELESCULPTOR_WORLDVIEW_H_

#include "EditMode.h"

#include <vital/config/config_block_types.h>
#include <vital/types/camera_map.h>
#include <vital/types/local_geo_cs.h>

#include <vtkSmartPointer.h>

#include <qtGlobal.h>

#include <QWidget>

class vtkBox;
class vtkImageData;
class vtkMaptkImageDataGeometryFilter;
class vtkObject;
class vtkPolyData;
class vtkStructuredGrid;
class MeshColoration;
namespace kwiver {
namespace arrows {
namespace vtk {
  class vtkKwiverCamera;
}}}

namespace kwiver {
namespace vital {
  class landmark_map;
}}

class GroundControlPointsWidget;
class RulerOptions;
class RulerWidget;

class WorldViewPrivate;

class WorldView : public QWidget
{
  Q_OBJECT

public:
  explicit WorldView(QWidget* parent = nullptr, Qt::WindowFlags flags = {});
  ~WorldView() override;

  void initFrameSampling(int nbFrames);

  bool loadVolume(QString const& path);

  void setVolume(vtkSmartPointer<vtkImageData> volume);

  void resetVolume();

  bool loadMesh(QString const& path);

  void setMesh(vtkSmartPointer<vtkPolyData> mesh);

  void setVideoConfig(QString const& path,
                      kwiver::vital::config_block_sptr config);
  void setMaskConfig(QString const& path,
                     kwiver::vital::config_block_sptr config);

  void enableAntiAliasing(bool enable);
  void setROI(vtkBox*, bool init = false);

  GroundControlPointsWidget* groundControlPointsWidget() const;
  RulerWidget* rulerWidget() const;
  void setRulerOptions(RulerOptions* r);

signals:
  void depthMapThresholdsChanged();
  void depthMapEnabled(bool);

  void contourChanged();
  void updateThresholds(double, double, double, double);
  void volumeEnabled(bool);
  void fusedMeshEnabled(bool);
  void pointPlacementEnabled(bool);
  void rulerEnabled(bool);

public slots:
  void setBackgroundColor(QColor const&);

  void addCamera(kwiver::vital::frame_id_t id,
                 kwiver::arrows::vtk::vtkKwiverCamera* camera);
  void updateCamera(kwiver::vital::frame_id_t id,
                    kwiver::arrows::vtk::vtkKwiverCamera* camera);
  void removeCamera(kwiver::vital::frame_id_t id);
  void setLandmarks(kwiver::vital::landmark_map const&);

  void setValidDepthInput(bool);
  void connectDepthPipeline();
  void setDepthGeometryFilter(vtkMaptkImageDataGeometryFilter*);
  void updateDepthMap();

  void setImageData(vtkImageData* data, QSize dimensions);

  void setImageVisible(bool);
  void setCamerasVisible(bool);
  void setLandmarksVisible(bool);
  void setGroundPlaneVisible(bool);
  void setAxesVisible(bool);
  void setDepthMapVisible(bool);
  void selectROI(bool);
  void resetROI();

  void setPerspective(bool);

  void setActiveCamera(kwiver::vital::frame_id_t id);

  void queueResetView();
  void resetView();
  void resetViewToLandmarks();

  void viewToWorldTop();
  void viewToWorldLeft();
  void viewToWorldRight();
  void viewToWorldFront();
  void viewToWorldBack();

  void setEditMode(EditMode);

  bool saveDepthPoints(QString const & path,
                       kwiver::vital::local_geo_cs const & lgcs);
  void exportWebGLScene(QString const& path);

  bool saveVolume(QString const& path);
  bool saveFusedMesh(QString const& path,
                     kwiver::vital::local_geo_cs const & lgcs);
  void saveFusedMeshFrameColors(QString const& path, bool occlusion = true);
  void meshColorationHandleResult(MeshColoration* coloration);

  void invalidateGeometry();
  void invalidateCameras();

  void setVolumeVisible(bool);
  void setVolumeCurrentFrame(kwiver::vital::frame_id_t);

  void computeContour(double threshold);
  void render();

protected slots:
  void updateAxes();
  void updateCameras();
  void updateScale();
  void updateDepthMapDisplayMode();
  void updateDepthMapThresholds(bool filterState);
  void increaseDepthMapPointSize();
  void decreaseDepthMapPointSize();
  void updateThresholdRanges();
  void updateROI(vtkObject*, unsigned long, void*, void*);

private:
  QTE_DECLARE_PRIVATE_RPTR(WorldView)
  QTE_DECLARE_PRIVATE(WorldView)

  QTE_DISABLE_COPY(WorldView)
};

#endif
