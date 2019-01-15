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
#include "vtkMaptkCamera.h"
#include "vtkMaptkPointPicker.h"
#include "vtkMaptkPointPlacer.h"

#include <vital/types/geodesy.h>

#include <vital/range/transform.h>

#include <vtkHandleWidget.h>
#include <vtkPlane.h>
#include <vtkRenderer.h>

#include <qtStlUtil.h>

#include <QDebug>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMessageBox>

namespace kv = kwiver::vital;
namespace kvr = kwiver::vital::range;

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

  // Check for valid coordinates
  auto const& coords = geom.value(TAG_COORDINATES).toArray();
  if (coords.size() < 2 || coords.size() > 3 || !isDoubleArray(coords))
  {
    qDebug() << "ignoring point feature" << f
             << "with bad or missing coordinate specification";
    return nullptr;
  }

  // Create point
  auto gcp = std::make_shared<kv::ground_control_point>();

  // Set world location and elevation; per the GeoJSON specification
  // (RFC 7946), the coordinates shall have been specified in WGS'84
  constexpr static auto gcs = kv::SRID::lat_lon_WGS84;
  gcp->set_geo_loc({{coords[0].toDouble(), coords[1].toDouble()}, gcs});

  if (coords.size() > 2)
  {
    gcp->set_elevation(coords[2].toDouble());
  }

  // Get properties
  auto const& props = f.value(TAG_PROPERTIES).toObject();
  gcp->set_name(stdString(props.value(TAG_NAME).toString()));

  auto const& loc = props.value(TAG_LOCATION).toArray();
  if (loc.size() == 3 && isDoubleArray(loc))
  {
    gcp->set_loc({loc[0].toDouble(), loc[1].toDouble(), loc[2].toDouble()});
  }
  else
  {
    qDebug() << "bad or missing scene location in point" << f;
  }

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
  auto const& wl = gcp.geo_loc().location(kv::SRID::lat_lon_WGS84);

  // Create geometry
  QJsonObject geom;
  geom.insert(TAG_TYPE, TAG_POINT);
  geom.insert(TAG_COORDINATES, QJsonArray{wl[0], wl[1], gcp.elevation()});

  // Create properties
  QJsonObject props;
  props.insert(TAG_NAME, qtString(gcp.name()));
  props.insert(TAG_LOCATION, QJsonArray{sl[0], sl[1], sl[2]});

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
  MainWindow* mainWindow = nullptr;
  id_t curId = 0;
  kv::ground_control_point_map::ground_control_point_map_t groundControlPoints;
  std::map<vtkHandleWidget*, id_t> gcpHandleIdMap;

  void movePoint(GroundControlPointsHelper* q,
                 int handleId,
                 GroundControlPointsWidget* widget,
                 kv::vector_3d const& newPosition);

  id_t addPoint();
  id_t removePoint(vtkHandleWidget*);
};

QTE_IMPLEMENT_D_FUNC(GroundControlPointsHelper)

//-----------------------------------------------------------------------------
void GroundControlPointsHelperPrivate::movePoint(
  GroundControlPointsHelper* q,
  int handleId,
  GroundControlPointsWidget* widget,
  kv::vector_3d const& newPosition)
{
  // Actually move the point and update the view
  widget->movePoint(handleId, newPosition[0], newPosition[1], newPosition[2]);
  widget->render();

  // Find the corresponding GCP ID
  GroundControlPointsWidget* const worldWidget =
    this->mainWindow->worldView()->groundControlPointsWidget();
  vtkHandleWidget* const handle = worldWidget->handleWidget(handleId);

  // Signal that the point changed, or report error if ID was not found
  try
  {
    id_t const gcpId = this->gcpHandleIdMap.at(handle);
    emit q->pointChanged(gcpId);
  }
  catch (std::out_of_range const&)
  {
    qWarning() << "Failed to find the ID associated with the handle widget"
               << handle << "with VTK ID" << handleId;
  }
}

//-----------------------------------------------------------------------------
id_t GroundControlPointsHelperPrivate::addPoint()
{
  GroundControlPointsWidget* worldWidget =
    this->mainWindow->worldView()->groundControlPointsWidget();
  auto const pt = worldWidget->activePoint();
  this->groundControlPoints[curId] =
    std::make_shared<kv::ground_control_point>(pt);
  vtkHandleWidget* const handle =
    worldWidget->handleWidget(worldWidget->activeHandle());
  this->gcpHandleIdMap[handle] = curId;
  return curId++;
}

//-----------------------------------------------------------------------------
id_t GroundControlPointsHelperPrivate::removePoint(vtkHandleWidget* handle)
{
  auto const& i = this->gcpHandleIdMap.find(handle);
  if (i == this->gcpHandleIdMap.end())
  {
    throw std::out_of_range{ __func__ };
  }

  this->gcpHandleIdMap.erase(i);
  this->groundControlPoints.erase(i->second);
  return i->second;
}

//-----------------------------------------------------------------------------
GroundControlPointsHelper::GroundControlPointsHelper(QObject* parent)
  : QObject(parent)
  , d_ptr(new GroundControlPointsHelperPrivate)
{
  QTE_D();

  d->mainWindow = qobject_cast<MainWindow*>(parent);
  Q_ASSERT(d->mainWindow);

  GroundControlPointsWidget* worldWidget =
    d->mainWindow->worldView()->groundControlPointsWidget();
  GroundControlPointsWidget* cameraWidget =
    d->mainWindow->cameraView()->groundControlPointsWidget();

  // Set a point placer on the world widget.
  // This has to be set before the widget is enabled.
  worldWidget->setPointPlacer(vtkNew<vtkMaptkPointPlacer>());

  // connections
  connect(worldWidget, &GroundControlPointsWidget::pointPlaced,
          this, &GroundControlPointsHelper::addCameraViewPoint);
  connect(cameraWidget, &GroundControlPointsWidget::pointPlaced,
          this, &GroundControlPointsHelper::addWorldViewPoint);

  connect(worldWidget, &GroundControlPointsWidget::pointMoved,
          this, &GroundControlPointsHelper::moveCameraViewPoint);
  connect(cameraWidget, &GroundControlPointsWidget::pointMoved,
          this, &GroundControlPointsHelper::moveWorldViewPoint);
  connect(worldWidget, &GroundControlPointsWidget::pointDeleted,
          this, &GroundControlPointsHelper::removePoint);
  connect(cameraWidget, &GroundControlPointsWidget::pointDeleted,
          this, &GroundControlPointsHelper::removePoint);
  connect(worldWidget, &GroundControlPointsWidget::pointDeleted,
          cameraWidget, &GroundControlPointsWidget::deletePoint);
  connect(cameraWidget, &GroundControlPointsWidget::pointDeleted,
          worldWidget, &GroundControlPointsWidget::deletePoint);
  connect(cameraWidget, &GroundControlPointsWidget::activePointChanged,
          worldWidget, &GroundControlPointsWidget::setActivePoint);
  connect(worldWidget, &GroundControlPointsWidget::activePointChanged,
          cameraWidget, &GroundControlPointsWidget::setActivePoint);
}

//-----------------------------------------------------------------------------
GroundControlPointsHelper::~GroundControlPointsHelper()
{
}

//-----------------------------------------------------------------------------
void GroundControlPointsHelper::addCameraViewPoint()
{
  QTE_D();

  vtkMaptkCamera* camera = d->mainWindow->activeCamera();
  if (!camera)
  {
    return;
  }

  GroundControlPointsWidget* worldWidget =
    d->mainWindow->worldView()->groundControlPointsWidget();
  kv::vector_3d p = worldWidget->activePoint();

  double cameraPt[2];
  camera->ProjectPoint(p, cameraPt);
  GroundControlPointsWidget* cameraWidget =
    d->mainWindow->cameraView()->groundControlPointsWidget();
  cameraWidget->addPoint(cameraPt[0], cameraPt[1], 0.0);
  cameraWidget->render();

  emit this->pointAdded(d->addPoint());
  emit this->pointCountChanged(d->groundControlPoints.size());
}

//-----------------------------------------------------------------------------
void GroundControlPointsHelper::addWorldViewPoint()
{
  QTE_D();

  vtkMaptkCamera* camera = d->mainWindow->activeCamera();
  if (!camera)
  {
    return;
  }

  GroundControlPointsWidget* cameraWidget =
    d->mainWindow->cameraView()->groundControlPointsWidget();
  kv::vector_3d cameraPt = cameraWidget->activePoint();
  // Use an arbitarily value for depth to ensure that the landmarks would
  // be between the camera center and the back-projected point.
  kv::vector_3d p =
    camera->UnprojectPoint(cameraPt.data(), 100 * camera->GetDistance());

  // Pick a point along the active camera direction and use the depth of the
  // point to back-project the camera view ground control point.
  GroundControlPointsWidget* worldWidget =
    d->mainWindow->worldView()->groundControlPointsWidget();
  vtkNew<vtkMaptkPointPicker> pointPicker;
  double distance = 0;
  double gOrigin[3] = { 0, 0, 0 };
  double gNormal[3] = { 0, 0, 1 };
  if (pointPicker->Pick3DPoint(
        camera->GetPosition(), p.data(), worldWidget->renderer()))
  {
    p = kwiver::vital::vector_3d(pointPicker->GetPickPosition());
    p = camera->UnprojectPoint(cameraPt.data(), camera->Depth(p));
  }
  else if (vtkPlane::IntersectWithLine(camera->GetPosition(),
                                       p.data(),
                                       gNormal,
                                       gOrigin,
                                       distance,
                                       p.data()))
  {
    // Find the point where the ray intersects the ground plane and use that.
  }
  else
  {
    // If nothing was picked, ensure that the back-projection uses the depth of
    // camera origin point
    p = camera->UnprojectPoint(cameraPt.data());
  }
  worldWidget->addPoint(p);
  worldWidget->render();

  emit this->pointAdded(d->addPoint());
  emit this->pointCountChanged(d->groundControlPoints.size());
}

//-----------------------------------------------------------------------------
void GroundControlPointsHelper::removePoint(int handleId)
{
  QTE_D();
  GroundControlPointsWidget* worldWidget =
    d->mainWindow->worldView()->groundControlPointsWidget();
  vtkHandleWidget* const handleWidget = worldWidget->handleWidget(handleId);
  if (!handleWidget)
  {
    qWarning() << "Failed to find the VTK handle widget with handle"
               << handleId;
    return;
  }
  try
  {
    auto const gcpId = d->removePoint(handleWidget);
    emit this->pointRemoved(gcpId);
    emit this->pointCountChanged(d->groundControlPoints.size());
  }
  catch (std::out_of_range const&)
  {
    qWarning() << "Failed to find the ID associated with the handle widget"
               << handleWidget << "with VTK ID" << handleId;
  }
}

//-----------------------------------------------------------------------------
void GroundControlPointsHelper::moveCameraViewPoint()
{
  QTE_D();

  vtkMaptkCamera* camera = d->mainWindow->activeCamera();
  if (!camera)
  {
    return;
  }

  GroundControlPointsWidget* worldWidget =
    d->mainWindow->worldView()->groundControlPointsWidget();
  int handleId = worldWidget->activeHandle();

  kv::vector_3d p = worldWidget->activePoint();

  double cameraPt[2];
  camera->ProjectPoint(p, cameraPt);
  GroundControlPointsWidget* cameraWidget =
    d->mainWindow->cameraView()->groundControlPointsWidget();
  d->movePoint(this, handleId, cameraWidget,
               { cameraPt[0], cameraPt[1], 0.0 });
}

//-----------------------------------------------------------------------------
void GroundControlPointsHelper::moveWorldViewPoint()
{
  QTE_D();

  vtkMaptkCamera* camera = d->mainWindow->activeCamera();
  if (!camera)
  {
    return;
  }

  GroundControlPointsWidget* cameraWidget =
    d->mainWindow->cameraView()->groundControlPointsWidget();
  GroundControlPointsWidget* worldWidget =
    d->mainWindow->worldView()->groundControlPointsWidget();

  int handleId = cameraWidget->activeHandle();

  kv::vector_3d cameraPt = cameraWidget->activePoint();
  kv::vector_3d pt = worldWidget->point(handleId);
  double depth = d->mainWindow->activeCamera()->Depth(pt);

  kv::vector_3d p = camera->UnprojectPoint(cameraPt.data(), depth);
  d->movePoint(this, handleId, worldWidget, p);
}

//-----------------------------------------------------------------------------
void GroundControlPointsHelper::updateCameraViewPoints()
{
  QTE_D();

  vtkMaptkCamera* camera = d->mainWindow->activeCamera();
  if (!camera)
  {
    return;
  }

  GroundControlPointsWidget* worldWidget =
    d->mainWindow->worldView()->groundControlPointsWidget();

  GroundControlPointsWidget* cameraWidget =
    d->mainWindow->cameraView()->groundControlPointsWidget();

  int numCameraPts = cameraWidget->numberOfPoints();
  int numWorldPts = worldWidget->numberOfPoints();
  while (numCameraPts > numWorldPts)
  {
    cameraWidget->deletePoint(--numCameraPts);
  }

  for (int i = 0; i < numWorldPts; ++i)
  {
    kv::vector_3d worldPt = worldWidget->point(i);
    double cameraPt[2];
    camera->ProjectPoint(worldPt, cameraPt);
    if (i >= numCameraPts)
    {
      cameraWidget->addPoint(cameraPt[0], cameraPt[1], 0.0);
    }
    else
    {
      cameraWidget->movePoint(i, cameraPt[0], cameraPt[1], 0.0);
    }
  }
}

//-----------------------------------------------------------------------------
void GroundControlPointsHelper::setGroundControlPoints(
  kv::ground_control_point_map const& gcpm)
{
  QTE_D();

  // Clear all existing ground control points before adding new ones.
  GroundControlPointsWidget* cameraWidget =
    d->mainWindow->cameraView()->groundControlPointsWidget();
  cameraWidget->clearPoints();
  GroundControlPointsWidget* worldWidget =
    d->mainWindow->worldView()->groundControlPointsWidget();
  worldWidget->clearPoints();

  auto const& groundControlPoints = gcpm.ground_control_points();
  for (auto const& gcp : groundControlPoints)
  {
    auto const& pos = gcp.second->loc();

    worldWidget->addPoint(pos);
    this->addCameraViewPoint();
  }
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

  // Read points from feature collection
  auto getJsonObject = [](QJsonValue const& v){ return v.toObject(); };
  for (auto const& f : features.toArray() | kvr::transform(getJsonObject))
  {
    if (f.value(TAG_TYPE).toString() != TAG_FEATURE)
    {
      qWarning() << "ignoring non-feature object" << f
                 << "in FeatureCollection";
      continue;
    }
    if (auto gcp = extractGroundControlPoint(f))
    {
      // TODO do something with point!
    }
  }

  return true;
}

//-----------------------------------------------------------------------------
bool GroundControlPointsHelper::writeGroundControlPoints(
  QString const& path, QWidget* dialogParent) const
{
  QTE_D();

  QJsonArray features;
  for (auto const& gcpi : d->groundControlPoints)
  {
    auto const& f = buildFeature(gcpi.second);
    if (f.isObject())
    {
      features.append(f);
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

  return true;
}

//-----------------------------------------------------------------------------
void GroundControlPointsHelper::enableWidgets(bool enable)
{
  QTE_D();
  d->mainWindow->worldView()->groundControlPointsWidget()->enableWidget(enable);
  d->mainWindow->cameraView()->groundControlPointsWidget()->enableWidget(
    enable);
}

//-----------------------------------------------------------------------------
kv::ground_control_point_sptr GroundControlPointsHelper::groundControlPoint(
  id_t pointId)
{
  QTE_D();
  auto it = d->groundControlPoints.find(pointId);
  return it == d->groundControlPoints.end() ? nullptr : it->second;
}
