// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#ifndef TELESCULPTOR_CAMERAVIEW_H_
#define TELESCULPTOR_CAMERAVIEW_H_

#include "EditMode.h"

#include <vital/vital_types.h>

#include <qtGlobal.h>

#include <QWidget>

class vtkImageData;

namespace kwiver {

namespace arrows {

namespace vtk {

class vtkKwiverCamera;

} // namespace vtk

} // namespace arrows

namespace vital {

class landmark_map;
class track;

} // namespace vital

} // namespace kwiver

class GroundControlPointsWidget;
class RulerWidget;

class CameraViewPrivate;

class CameraView : public QWidget
{
  Q_OBJECT

public:
  explicit CameraView(QWidget* parent = nullptr, Qt::WindowFlags flags = {});
  ~CameraView() override;

  void addFeatureTrack(kwiver::vital::track const&);
  GroundControlPointsWidget* groundControlPointsWidget() const;
  GroundControlPointsWidget* registrationPointsWidget() const;
  RulerWidget* rulerWidget() const;

  void enableAntiAliasing(bool enable);

signals:
  void pointPlacementEnabled(bool);
  void cameraComputationRequested();

public slots:
  void setBackgroundColor(QColor const&);

  void setImagePath(QString const&);
  void setImageData(vtkImageData* data, QSize dimensions);

  void setLandmarksData(kwiver::vital::landmark_map const&);

  void setEditMode(EditMode);

  void setActiveCamera(kwiver::arrows::vtk::vtkKwiverCamera*);
  void setActiveFrame(kwiver::vital::frame_id_t);

  void addLandmark(kwiver::vital::landmark_id_t id, double x, double y);
  void addResidual(kwiver::vital::track_id_t id,
                   double x1, double y1,
                   double x2, double y2,
                   bool inlier);

  void clearLandmarks();
  void clearResiduals();
  void clearFeatureTracks();
  void clearGroundControlPoints();

  void resetView();
  void resetViewToFullExtents();
  void render();

protected slots:
  void setImageVisible(bool);
  void setLandmarksVisible(bool);
  void setResidualsVisible(bool);
  void setOutlierResidualsVisible(bool);

  void updateFeatures();

private:
  QTE_DECLARE_PRIVATE_RPTR(CameraView)
  QTE_DECLARE_PRIVATE(CameraView)

  QTE_DISABLE_COPY(CameraView)
};

#endif
