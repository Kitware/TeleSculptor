/*ckwg +29
 * Copyright 2018-2019 by Kitware, Inc.
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

#include "GroundControlPointsHelper.h"

#include "CameraView.h"
#include "GroundControlPointsWidget.h"
#include "MainWindow.h"
#include "WorldView.h"
#include "vtkMaptkPointPicker.h"
#include "vtkMaptkPointPlacer.h"

#include "arrows/vtk/vtkKwiverCamera.h"
#include <vital/types/geodesy.h>

#include <vtkHandleWidget.h>
#include <vtkPlane.h>
#include <vtkRenderer.h>

#include <qtGet.h>
#include <qtScopedValueChange.h>
#include <qtStlUtil.h>

#include <QDebug>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMessageBox>

namespace kv = kwiver::vital;

using id_t = kv::ground_control_point_id_t;

namespace
{

// Keys
const auto TAG_TYPE               = QStringLiteral("type");
const auto TAG_FEATURES           = QStringLiteral("features");
const auto TAG_GEOMETRY           = QStringLiteral("geometry");
const auto TAG_PROPERTIES         = QStringLiteral("properties");
const auto TAG_COORDINATES        = QStringLiteral("coordinates");

// Values
const auto TAG_FEATURE            = QStringLiteral("Feature");
const auto TAG_FEATURECOLLECTION  = QStringLiteral("FeatureCollection");
const auto TAG_POINT              = QStringLiteral("Point");

// Property keys (not part of GeoJSON specification)
const auto TAG_NAME               = QStringLiteral("name");
const auto TAG_LOCATION           = QStringLiteral("location");
const auto TAG_USER_REGISTERED    = QStringLiteral("userRegistered");

enum class Reset
{
  Force = 1<<0,
  Silent = 1<<1,
};

Q_DECLARE_FLAGS(ResetFlags, Reset)
Q_DECLARE_OPERATORS_FOR_FLAGS(ResetFlags)

//-----------------------------------------------------------------------------
bool isDoubleArray(QJsonArray const& a)
{
  for (auto const& v : a)
  {
    if (!v.isDouble())
    {
      return false;
    }
  }
  return true;
}

//-----------------------------------------------------------------------------
kv::ground_control_point_sptr extractGroundControlPoint(QJsonObject const& f)
{
  // Check for geometry
  auto const& geom = f.value(TAG_GEOMETRY).toObject();
  if (geom.isEmpty())
  {
    qDebug() << "ignoring feature" << f
             << "with bad or missing geometry";
    return nullptr;
  }

  // Check geometry type
  if (geom.value(TAG_TYPE).toString() != TAG_POINT)
  {
    // Non-point features are silently ignored
    return nullptr;
  }

  // Create point
  auto gcp = std::make_shared<kv::ground_control_point>();

  // Check for valid coordinates
  auto const& coords = geom.value(TAG_COORDINATES).toArray();
  if (coords.size() >= 2 && coords.size() <= 3 && isDoubleArray(coords))
  {
    // Set world location and elevation; per the GeoJSON specification
    // (RFC 7946), the coordinates shall have been specified in WGS'84
    constexpr static auto gcs = kv::SRID::lat_lon_WGS84;
    kv::vector_3d loc(coords[0].toDouble(), coords[1].toDouble(), 0.0);
    if (coords.size() > 2)
    {
      loc[2] = coords[2].toDouble();
    }
    gcp->set_geo_loc({loc, gcs});
  }

  // Get properties
  auto const& props = f.value(TAG_PROPERTIES).toObject();
  gcp->set_name(stdString(props.value(TAG_NAME).toString()));

  auto const& loc = props.value(TAG_LOCATION).toArray();
  if (loc.size() == 3 && isDoubleArray(loc))
  {
    gcp->set_loc({loc[0].toDouble(), loc[1].toDouble(), loc[2].toDouble()});
  }
  else if (gcp->geo_loc().is_empty())
  {
    qDebug() << "ignoring point feature" << f
             << "with bad or missing coordinate specification "
                "and bad or missing scene location";
    return nullptr;
  }

  gcp->set_geo_loc_user_provided(props.value(TAG_USER_REGISTERED).toBool());

  return gcp;
}

//-----------------------------------------------------------------------------
QJsonValue buildFeature(kv::ground_control_point_sptr const& gcpp)
{
  if (!gcpp)
  {
    return {};
  }

  // Get point and point's locations (scene, world/geodetic)
  auto const& gcp = *gcpp;
  auto const& sl = gcp.loc();
  auto const& wl = gcp.geo_loc();

  // Create geometry
  QJsonObject geom;
  geom.insert(TAG_TYPE, TAG_POINT);
  if (!wl.is_empty())
  {
    auto const& rwl = wl.location(kv::SRID::lat_lon_WGS84);
    geom.insert(TAG_COORDINATES, QJsonArray{rwl[0], rwl[1], gcp.elevation()});
  }

  // Create properties
  QJsonObject props;
  props.insert(TAG_NAME, qtString(gcp.name()));
  props.insert(TAG_LOCATION, QJsonArray{sl[0], sl[1], sl[2]});
  props.insert(TAG_USER_REGISTERED, gcp.is_geo_loc_user_provided());

  // Create and return feature
  QJsonObject f;
  f.insert(TAG_TYPE, TAG_FEATURE);
  f.insert(TAG_GEOMETRY, geom);
  f.insert(TAG_PROPERTIES, props);

  return f;
}

} // namespace (anonymous)

//-----------------------------------------------------------------------------
class GroundControlPointsHelperPrivate
{
public:
  GroundControlPointsHelperPrivate(GroundControlPointsHelper* q) : q_ptr{q} {}

  MainWindow* mainWindow = nullptr;
  GroundControlPointsWidget* worldWidget = nullptr;
  GroundControlPointsWidget* cameraWidget = nullptr;

  id_t nextId = 0;
  kv::ground_control_point_map::ground_control_point_map_t groundControlPoints;
  std::map<vtkHandleWidget*, id_t> gcpHandleToIdMap;
  std::map<id_t, vtkHandleWidget*> gcpIdToHandleMap;

  void addPoint(id_t id, kv::ground_control_point_sptr const& point);

  void updatePoint(int handleId);
  void movePoint(int handleId, GroundControlPointsWidget* widget,
                 kv::vector_3d const& newPosition);

  void resetPoint(id_t gcpId, ResetFlags options = {});
  void resetPoint(kv::ground_control_point_sptr const& gcp,
                  id_t gcpId, ResetFlags options = {});

  void removePoint(int handleId, vtkHandleWidget* handleWidget);

  void updateActivePoint(int handleId);

  bool addCameraViewPoint();

  id_t addPoint();

private:
  QTE_DECLARE_PUBLIC_PTR(GroundControlPointsHelper)
  QTE_DECLARE_PUBLIC(GroundControlPointsHelper)
};

QTE_IMPLEMENT_D_FUNC(GroundControlPointsHelper)

//-----------------------------------------------------------------------------
void GroundControlPointsHelperPrivate::addPoint(
  id_t id, kv::ground_control_point_sptr const& point)
{
  // Add point to internal map
  this->groundControlPoints[id] = point;

  // Add point to VTK widgets
  auto const& pos = point->loc();
  this->worldWidget->addPoint(pos);
  this->addCameraViewPoint();

  // Add handle to handle map
  auto* const handle =
    this->worldWidget->handleWidget(this->worldWidget->activeHandle());
  this->gcpHandleToIdMap[handle] = id;
  this->gcpIdToHandleMap[id] = handle;
}

//-----------------------------------------------------------------------------
void GroundControlPointsHelperPrivate::resetPoint(
  id_t gcpId, ResetFlags options)
{
  auto const gcpp = qtGet(this->groundControlPoints, gcpId);
  auto const gcp = (gcpp ? gcpp->second : nullptr);

  if (!gcp)
  {
    qWarning() << "No ground control point with id" << gcpId;
    return;
  }

  this->resetPoint(gcp, gcpId, options);
}

//-----------------------------------------------------------------------------
void GroundControlPointsHelperPrivate::resetPoint(
  kv::ground_control_point_sptr const& gcp, id_t gcpId, ResetFlags options)
{
  if ((options & Reset::Force) || !gcp->is_geo_loc_user_provided())
  {
    QTE_Q();

    gcp->set_geo_loc_user_provided(false);
    auto lgcs = this->mainWindow->localGeoCoordinateSystem();
    if (lgcs.origin().crs() >= 0)
    {
      kwiver::vital::vector_3d loc(lgcs.origin().location() + gcp->loc());
      gcp->set_geo_loc({loc, lgcs.origin().crs()});
    }

    if (!(options & Reset::Silent))
    {
      emit q->pointChanged(gcpId);
    }
  }
}

//-----------------------------------------------------------------------------
void GroundControlPointsHelperPrivate::updatePoint(int handleId)
{
  // Find the corresponding GCP ID
  auto* const handle = this->worldWidget->handleWidget(handleId);

  if (auto const gcpIdIter = qtGet(this->gcpHandleToIdMap, handle))
  {
    // Find the corresponding GCP
    auto const gcpId = gcpIdIter->second;
    auto const gcpIter = qtGet(this->groundControlPoints, gcpId);
    auto const gcp = (gcpIter ? gcpIter->second : nullptr);

    if (gcp)
    {
      // Update the GCP's scene location
      gcp->set_loc(this->worldWidget->point(handleId));

      // (Possibly) reset the point's geodetic location and signal the change
      this->resetPoint(gcpId);
    }
    else
    {
      qWarning() << "No ground control point with id" << gcpId;
    }
  }
  else
  {
    qWarning() << "Failed to find the ID associated with the handle widget"
               << handle << "with VTK ID" << handleId;
  }
}

//-----------------------------------------------------------------------------
void GroundControlPointsHelperPrivate::movePoint(
  int handleId, GroundControlPointsWidget* widget,
  kv::vector_3d const& newPosition)
{
  // Actually move the point and update the view
  widget->movePoint(handleId, newPosition[0], newPosition[1], newPosition[2]);
  widget->render();

  this->updatePoint(handleId);
}

//-----------------------------------------------------------------------------
void GroundControlPointsHelperPrivate::removePoint(
  int handleId, vtkHandleWidget* handleWidget)
{
  auto const& i = this->gcpHandleToIdMap.find(handleWidget);
  if (i == this->gcpHandleToIdMap.end())
  {
    qWarning() << "Failed to find the ID associated with the handle widget"
               << handleWidget << "with VTK ID" << handleId;
    return;
  }

  auto const gcpId = i->second;
  this->gcpHandleToIdMap.erase(i);
  this->gcpIdToHandleMap.erase(gcpId);
  this->groundControlPoints.erase(gcpId);

  QTE_Q();

  emit q->pointRemoved(gcpId);
  emit q->pointCountChanged(this->groundControlPoints.size());
}

//-----------------------------------------------------------------------------
id_t GroundControlPointsHelperPrivate::addPoint()
{
  auto const pt = this->worldWidget->activePoint();
  this->groundControlPoints[nextId] =
    std::make_shared<kv::ground_control_point>(pt);
  vtkHandleWidget* const handle =
    this->worldWidget->handleWidget(this->worldWidget->activeHandle());

  this->gcpHandleToIdMap[handle] = nextId;
  this->gcpIdToHandleMap[nextId] = handle;

  this->resetPoint(nextId, Reset::Silent);

  return nextId++;
}

//-----------------------------------------------------------------------------
void GroundControlPointsHelperPrivate::updateActivePoint(int handleId)
{
  QTE_Q();

  // Find the corresponding GCP ID
  vtkHandleWidget* const handle = this->worldWidget->handleWidget(handleId);

  // Signal that the point changed, or report error if ID was not found
  try
  {
    emit q->activePointChanged(this->gcpHandleToIdMap.at(handle));
  }
  catch (std::out_of_range const&)
  {
    qWarning() << "Failed to find the ID associated with the handle widget"
               << handle << "with VTK ID" << handleId;
  }
}

//-----------------------------------------------------------------------------
bool GroundControlPointsHelperPrivate::addCameraViewPoint()
{
  auto* const camera = this->mainWindow->activeCamera();
  if (!camera)
  {
    return false;
  }

  kv::vector_3d p = this->worldWidget->activePoint();

  double cameraPt[2];
  bool valid = camera->ProjectPoint(p, cameraPt);
  this->cameraWidget->addPoint(cameraPt[0], cameraPt[1], 0.0);
  this->cameraWidget->render();
  return valid;
}

//-----------------------------------------------------------------------------
GroundControlPointsHelper::GroundControlPointsHelper(QObject* parent)
  : QObject{parent}, d_ptr{new GroundControlPointsHelperPrivate{this}}
{
  QTE_D();

  d->mainWindow = qobject_cast<MainWindow*>(parent);
  Q_ASSERT(d->mainWindow);

  d->worldWidget = d->mainWindow->worldView()->groundControlPointsWidget();
  d->cameraWidget = d->mainWindow->cameraView()->groundControlPointsWidget();

  // Set a point placer on the world widget.
  // This has to be set before the widget is enabled.
  d->worldWidget->setPointPlacer(vtkNew<vtkMaptkPointPlacer>());

  // connections
  connect(d->worldWidget, &GroundControlPointsWidget::pointPlaced,
          this, &GroundControlPointsHelper::addCameraViewPoint);
  connect(d->cameraWidget, &GroundControlPointsWidget::pointPlaced,
          this, &GroundControlPointsHelper::addWorldViewPoint);
  connect(d->worldWidget, &GroundControlPointsWidget::pointMoved,
          this, &GroundControlPointsHelper::moveCameraViewPoint);
  connect(d->cameraWidget, &GroundControlPointsWidget::pointMoved,
          this, &GroundControlPointsHelper::moveWorldViewPoint);
  connect(d->worldWidget, &GroundControlPointsWidget::pointDeleted,
          this, &GroundControlPointsHelper::removePointByHandle);
  connect(d->cameraWidget, &GroundControlPointsWidget::pointDeleted,
          this, &GroundControlPointsHelper::removePointByHandle);
  connect(d->worldWidget, &GroundControlPointsWidget::activePointChanged,
          this, [d](int handleId){ d->updateActivePoint(handleId); });

  connect(d->worldWidget, &GroundControlPointsWidget::pointDeleted,
          d->cameraWidget, &GroundControlPointsWidget::deletePoint);
  connect(d->cameraWidget, &GroundControlPointsWidget::pointDeleted,
          d->worldWidget, &GroundControlPointsWidget::deletePoint);
  connect(d->cameraWidget, &GroundControlPointsWidget::activePointChanged,
          d->worldWidget, &GroundControlPointsWidget::setActivePoint);
  connect(d->worldWidget, &GroundControlPointsWidget::activePointChanged,
          d->cameraWidget, &GroundControlPointsWidget::setActivePoint);
}

//-----------------------------------------------------------------------------
GroundControlPointsHelper::~GroundControlPointsHelper()
{
}

//-----------------------------------------------------------------------------
void GroundControlPointsHelper::addCameraViewPoint()
{
  QTE_D();

  d->addCameraViewPoint();

  emit this->pointAdded(d->addPoint());
  emit this->pointCountChanged(d->groundControlPoints.size());
}

//-----------------------------------------------------------------------------
void GroundControlPointsHelper::addWorldViewPoint()
{
  QTE_D();

  kwiver::arrows::vtk::vtkKwiverCamera* camera = d->mainWindow->activeCamera();
  if (!camera)
  {
    return;
  }

  kv::vector_3d cameraPt = d->cameraWidget->activePoint();
  // Use an arbitarily value for depth to ensure that the landmarks would
  // be between the camera center and the back-projected point.
  kv::vector_3d p =
    camera->UnprojectPoint(cameraPt.data(), 100 * camera->GetDistance());

  // Pick a point along the active camera direction and use the depth of the
  // point to back-project the camera view ground control point.
  vtkNew<vtkMaptkPointPicker> pointPicker;
  double distance = 0;
  double gOrigin[3] = { 0, 0, 0 };
  double gNormal[3] = { 0, 0, 1 };
  if (pointPicker->Pick3DPoint(
        camera->GetPosition(), p.data(), d->worldWidget->renderer()))
  {
    p = kwiver::vital::vector_3d(pointPicker->GetPickPosition());
    p = camera->UnprojectPoint(cameraPt.data(), camera->Depth(p));
  }
  else if (vtkPlane::IntersectWithLine(camera->GetPosition(), p.data(),
                                       gNormal, gOrigin, distance, p.data()))
  {
    // Find the point where the ray intersects the ground plane and use that.
  }
  else
  {
    // If nothing was picked, ensure that the back-projection uses the depth of
    // camera origin point
    p = camera->UnprojectPoint(cameraPt.data());
  }
  d->worldWidget->addPoint(p);
  d->worldWidget->render();

  emit this->pointAdded(d->addPoint());
  emit this->pointCountChanged(d->groundControlPoints.size());
}

//-----------------------------------------------------------------------------
void GroundControlPointsHelper::removePointByHandle(int handleId)
{
  QTE_D();

  auto* const handleWidget = d->worldWidget->handleWidget(handleId);
  if (!handleWidget)
  {
    qWarning() << "Failed to find the VTK handle widget with VTK ID"
               << handleId;
    return;
  }

  d->removePoint(handleId, handleWidget);
}

//-----------------------------------------------------------------------------
void GroundControlPointsHelper::moveCameraViewPoint()
{
  QTE_D();

  auto const handleId = d->worldWidget->activeHandle();

  kwiver::arrows::vtk::vtkKwiverCamera* camera = d->mainWindow->activeCamera();
  if (!camera)
  {
    d->updatePoint(handleId);
    return;
  }

  kv::vector_3d p = d->worldWidget->activePoint();

  double cameraPt[2];
  camera->ProjectPoint(p, cameraPt);
  d->movePoint(handleId, d->cameraWidget, { cameraPt[0], cameraPt[1], 0.0 });
}

//-----------------------------------------------------------------------------
void GroundControlPointsHelper::moveWorldViewPoint()
{
  QTE_D();

  kwiver::arrows::vtk::vtkKwiverCamera* camera = d->mainWindow->activeCamera();
  if (!camera)
  {
    return;
  }

  auto const handleId = d->cameraWidget->activeHandle();

  kv::vector_3d cameraPt = d->cameraWidget->activePoint();
  kv::vector_3d pt = d->worldWidget->point(handleId);
  double depth = d->mainWindow->activeCamera()->Depth(pt);

  kv::vector_3d p = camera->UnprojectPoint(cameraPt.data(), depth);
  d->movePoint(handleId, d->worldWidget, p);
}

//-----------------------------------------------------------------------------
void GroundControlPointsHelper::updateCameraViewPoints()
{
  QTE_D();

  kwiver::arrows::vtk::vtkKwiverCamera* camera = d->mainWindow->activeCamera();
  if (!camera)
  {
    return;
  }

  with_expr (qtScopedBlockSignals{d->cameraWidget})
  {
    auto const activeHandle = d->worldWidget->activeHandle();

    int numCameraPts = d->cameraWidget->numberOfPoints();
    int numWorldPts = d->worldWidget->numberOfPoints();
    while (numCameraPts > numWorldPts)
    {
      d->cameraWidget->deletePoint(--numCameraPts);
    }

    for (int i = 0; i < numWorldPts; ++i)
    {
      kv::vector_3d worldPt = d->worldWidget->point(i);
      double cameraPt[2];
      camera->ProjectPoint(worldPt, cameraPt);
      if (i >= numCameraPts)
      {
        d->cameraWidget->addPoint(cameraPt[0], cameraPt[1], 0.0);
      }
      else
      {
        d->cameraWidget->movePoint(i, cameraPt[0], cameraPt[1], 0.0);
      }
    }

    d->cameraWidget->setActivePoint(activeHandle);
  }
}

//-----------------------------------------------------------------------------
// Call this function when the GCP data has been modified
void GroundControlPointsHelper::updateViewsFromGCPs()
{
  QTE_D();

  kwiver::arrows::vtk::vtkKwiverCamera* camera = d->mainWindow->activeCamera();

  for (auto const& i : d->groundControlPoints)
  {
    auto* const hp = qtGet(d->gcpIdToHandleMap, i.first);
    auto* const handleWidget = (hp ? hp->second : nullptr);

    if (handleWidget)
    {
      auto const handleId = d->worldWidget->findHandleWidget(handleWidget);
      kwiver::vital::vector_3d loc = i.second->loc();
      d->worldWidget->movePoint(handleId, loc.x(), loc.y(), loc.z());

      if (camera)
      {
        double cameraPt[2];
        camera->ProjectPoint(loc, cameraPt);
        d->cameraWidget->movePoint(handleId, cameraPt[0], cameraPt[1], 0.0);
      }
    }
  }
  d->worldWidget->render();
  d->cameraWidget->render();
}

//-----------------------------------------------------------------------------
void GroundControlPointsHelper::recomputePoints()
{
  QTE_D();

  for (auto const& i : d->groundControlPoints)
  {
    d->resetPoint(i.second, i.first, Reset::Silent);
  }

  emit this->pointsRecomputed();
}

//-----------------------------------------------------------------------------
void GroundControlPointsHelper::resetPoint(id_t gcpId)
{
  QTE_D();
  d->resetPoint(gcpId, Reset::Force);
}

//-----------------------------------------------------------------------------
void GroundControlPointsHelper::removePoint(id_t gcpId)
{
  QTE_D();

  auto* const hp = qtGet(d->gcpIdToHandleMap, gcpId);
  auto* const handleWidget = (hp ? hp->second : nullptr);

  if (handleWidget)
  {
    auto const handleId = d->worldWidget->findHandleWidget(handleWidget);

    if (handleId < 0)
    {
      qWarning() << "Failed to find the VTK ID associated with the VTK handle"
                 << handleId << " and the ground control point with ID"
                 << gcpId;
    }
    else
    {
      d->worldWidget->deletePoint(handleId);
      d->cameraWidget->deletePoint(handleId);
      d->removePoint(handleId, handleWidget);
    }
  }
  else
  {
    qWarning() << "Failed to find the VTK handle associated with "
                  "the ground control point with ID" << gcpId;
  }
}

//-----------------------------------------------------------------------------
void GroundControlPointsHelper::setActivePoint(id_t gcpId)
{
  QTE_D();

  auto* const hp = qtGet(d->gcpIdToHandleMap, gcpId);
  auto* const handleWidget = (hp ? hp->second : nullptr);

  if (handleWidget)
  {
    auto const handleId = d->worldWidget->findHandleWidget(handleWidget);

    if (handleId < 0)
    {
      qWarning() << "Failed to find the VTK ID associated with the VTK handle"
                 << handleId << " and the ground control point with ID"
                 << gcpId;
    }
    else
    {
      d->worldWidget->setActivePoint(handleId);
      d->cameraWidget->setActivePoint(handleId);
    }
  }
  else
  {
    qWarning() << "Failed to find the VTK handle associated with "
                  "the ground control point with ID" << gcpId;
  }
}

//-----------------------------------------------------------------------------
void GroundControlPointsHelper::applySimilarityTransform()
{
  QTE_D();

  d->mainWindow->applySimilarityTransform();
}

//-----------------------------------------------------------------------------
void GroundControlPointsHelper::setGroundControlPoints(
  kv::ground_control_point_map const& gcpm)
{
  QTE_D();

  // Don't signal addition of individual points
  with_expr (qtScopedBlockSignals{this})
  {
    // Clear all existing ground control points before adding new ones.
    d->cameraWidget->clearPoints();
    d->worldWidget->clearPoints();
    d->groundControlPoints.clear();
    d->nextId = 0;

    auto const& groundControlPoints = gcpm.ground_control_points();
    for (auto const& gcp : groundControlPoints)
    {
      d->addPoint(gcp.first, gcp.second);
      d->nextId = std::max(gcp.first + 1, d->nextId);
    }
  }

  emit this->pointsReloaded();
  emit this->pointCountChanged(d->groundControlPoints.size());
}

//-----------------------------------------------------------------------------
kv::ground_control_point_map::ground_control_point_map_t const&
GroundControlPointsHelper::groundControlPoints() const
{
  QTE_D();
  return d->groundControlPoints;
}

//-----------------------------------------------------------------------------
bool GroundControlPointsHelper::readGroundControlPoints(QString const& path)
{
  QTE_D();

  auto fail = [&path](char const* extra){
    qWarning().nospace()
      << "failed to read ground control points from "
      << path << ": " << extra;
    return false;
  };

  // Open input file
  QFile f{path};
  if (!f.open(QIODevice::ReadOnly | QIODevice::Text))
  {
    return fail(qPrintable(f.errorString()));
  }

  // Read raw JSON data
  QJsonParseError err;
  auto const& doc = QJsonDocument::fromJson(f.readAll(), &err);
  if (doc.isNull())
  {
    return fail(qPrintable(err.errorString()));
  }
  if (!doc.isObject())
  {
    return fail("file does not contain a JSON object");
  }

  // Get feature collection and features
  auto const& collection = doc.object();
  if (collection.value(TAG_TYPE).toString() != TAG_FEATURECOLLECTION)
  {
    return fail("root object must be a FeatureCollection");
  }
  auto const& features = collection.value(TAG_FEATURES);
  if (!features.isArray())
  {
    return fail("invalid FeatureCollection");
  }


  // Don't signal addition of individual points
  auto pointsAdded = false;
  with_expr (qtScopedBlockSignals{this})
  {
    // Read points from feature collection
    for (auto const& f : features.toArray())
    {
      auto fo = f.toObject();
      if (fo.value(TAG_TYPE).toString() != TAG_FEATURE)
      {
        qWarning() << "ignoring non-feature object" << f
                   << "in FeatureCollection";
        continue;
      }
      if (auto gcp = extractGroundControlPoint(fo))
      {
        d->addPoint(d->nextId++, gcp);
        pointsAdded = true;
      }
    }
  }

  // Signal that points were changed
  if (pointsAdded)
  {
    emit this->pointsReloaded();
    emit this->pointCountChanged(d->groundControlPoints.size());
  }

  return true;
}

//-----------------------------------------------------------------------------
bool GroundControlPointsHelper::writeGroundControlPoints(
  QString const& path, QWidget* dialogParent) const
{
  QTE_D();

  QJsonArray features;
  QStringList errors;
  for (auto const& gcpi : d->groundControlPoints)
  {
    try
    {
      auto const& f = buildFeature(gcpi.second);
      if (f.isObject())
      {
        features.append(f);
      }
    }
    catch (std::exception const& e)
    {
      errors.append(QString::fromLocal8Bit(e.what()));
    }
  }

  QJsonObject root;
  root.insert(TAG_TYPE, TAG_FEATURECOLLECTION);
  root.insert(TAG_FEATURES, features);

  QJsonDocument doc{root};

  QFile out{path};
  if (!out.open(QIODevice::WriteOnly) || out.write(doc.toJson()) < 0)
  {
    auto const msg =
      QStringLiteral("An error occurred while exporting "
                     "ground control points to \"%1\": %2 ");
    QMessageBox::critical(dialogParent, QStringLiteral("Export error"),
                          msg.arg(path, out.errorString()));
    return false;
  }

  if (!errors.isEmpty())
  {
    auto const msg =
      QStringLiteral("One or more errors occurred while exporting "
                     "ground control points to \"%1\". "
                     "The output file may not have been written correctly.");

    QMessageBox mb{dialogParent};
    mb.setIcon(QMessageBox::Warning);
    mb.setText(msg.arg(path));
    mb.setDetailedText(errors.join('\n'));
    mb.setWindowTitle(QStringLiteral("Export error"));
    mb.exec();
  }

  return true;
}

//-----------------------------------------------------------------------------
void GroundControlPointsHelper::enableWidgets(bool enable)
{
  QTE_D();
  d->worldWidget->enableWidget(enable);
  d->cameraWidget->enableWidget(enable);
}

//-----------------------------------------------------------------------------
kv::ground_control_point_sptr GroundControlPointsHelper::groundControlPoint(
  id_t pointId)
{
  QTE_D();
  auto it = d->groundControlPoints.find(pointId);
  return it == d->groundControlPoints.end() ? nullptr : it->second;
}
