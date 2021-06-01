// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#ifndef TELESCULPTOR_GROUNDCONTROLPOINTSHELPER_H_
#define TELESCULPTOR_GROUNDCONTROLPOINTSHELPER_H_

#include <maptk/ground_control_point.h>

#include <vital/types/feature_track_set.h>
#include <vital/types/landmark_map.h>

#include <qtGlobal.h>

#include <QObject>

// Forward declarations
class GroundControlPointsHelperPrivate;

class GroundControlPointsHelper : public QObject
{
  Q_OBJECT

public:
  using id_t = kwiver::vital::ground_control_point_id_t;
  using gcp_sptr = kwiver::vital::ground_control_point_sptr;
  using crt_sptr = kwiver::vital::track_sptr;

  GroundControlPointsHelper(QObject* parent = nullptr);
  ~GroundControlPointsHelper();

  void updateCameraViewPoints();

  // Call this function when the GCP data has been modified
  void updateViewsFromGCPs();

  bool isEmpty() const;
  std::vector<id_t> identifiers() const;

  // Set ground control points
  void setGroundControlPoints(kwiver::vital::ground_control_point_map const&);

  // Get ground control points
  std::vector<gcp_sptr> groundControlPoints() const;

  // Get camera registration points
  kwiver::vital::feature_track_set_sptr registrationTracks() const;

  // Get ground control points as landmarks
  kwiver::vital::landmark_map_sptr registrationLandmarks() const;

  // Get access to a single ground control point
  gcp_sptr groundControlPoint(id_t pointId);

  // Get access to a camera registration "track"
  crt_sptr registrationTrack(id_t pointId);

  bool readGroundControlPoints(QString const& path);

  bool writeGroundControlPoints(
    QString const& path, QWidget* dialogParent) const;

public slots:
  void recomputePoints();

  void resetPoint(id_t);
  void removePoint(id_t);

  void setActivePoint(id_t);

  void applySimilarityTransform();

signals:
  void pointsReloaded();
  void pointsRecomputed();
  void pointCountChanged(size_t);
  void pointAdded(id_t);
  void pointRemoved(id_t);
  void pointChanged(id_t);

  void activePointChanged(id_t);

protected slots:
  void addRegistrationPoint();
  void addCameraViewPoint();
  void addWorldViewPoint();

  void removeCrpByHandle(int handleId);
  void removeGcpByHandle(int handleId);

  void moveRegistrationPoint();
  void moveCameraViewPoint();
  void moveWorldViewPoint();

private:
  QTE_DECLARE_PRIVATE_RPTR(GroundControlPointsHelper)
  QTE_DECLARE_PRIVATE(GroundControlPointsHelper)

  QTE_DISABLE_COPY(GroundControlPointsHelper)
};

#endif
