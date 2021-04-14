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

#ifndef TELESCULPTOR_WORLDVIEW_H_
#define TELESCULPTOR_WORLDVIEW_H_

#include <vital/config/config_block_types.h>
#include <vital/types/camera_map.h>
#include <vital/types/local_geo_cs.h>

#include <qtGlobal.h>

#include <QWidget>
#include <vtkSmartPointer.h>

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

  void loadVolume(QString const& path);

  void setVolume(vtkSmartPointer<vtkImageData> volume);

  void resetVolume();

  void loadMesh(QString const& path);

  void setMesh(vtkSmartPointer<vtkPolyData> mesh);

  void setVideoConfig(QString const& path,
                      kwiver::vital::config_block_sptr config);
  void setMaskConfig(QString const& path,
                     kwiver::vital::config_block_sptr config);
  void setCameras(kwiver::vital::camera_map_sptr cameras);

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


  void saveDepthPoints(QString const & path,
                       kwiver::vital::local_geo_cs const & lgcs);
  void exportWebGLScene(QString const& path);

  void saveVolume(QString const& path);
  void saveFusedMesh(QString const& path,
                     kwiver::vital::local_geo_cs const & lgcs);
  void saveFusedMeshFrameColors(QString const& path, bool occlusion = true);
  void meshColorationHandleResult(MeshColoration* coloration);

  void invalidateGeometry();

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
