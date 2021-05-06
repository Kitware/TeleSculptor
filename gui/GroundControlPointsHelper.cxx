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

#include <arrows/vtk/vtkKwiverCamera.h>

#include <vital/types/feature_track_set.h>
#include <vital/types/geodesy.h>

#include <vtkHandleWidget.h>
#include <vtkPlane.h>
#include <vtkRenderer.h>

#include <qtGet.h>
#include <qtIndexRange.h>
#include <qtScopedValueChange.h>
#include <qtStlUtil.h>

#include <QDebug>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMessageBox>

namespace kv = kwiver::vital;

namespace
{

constexpr int INVALID_HANDLE = -1;

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
const auto TAG_FRAME              = QStringLiteral("frameId");
const auto TAG_FRAMES             = QStringLiteral("frames");
const auto TAG_LOCATION           = QStringLiteral("location");
const auto TAG_REGISTRATIONS      = QStringLiteral("registrations");
const auto TAG_USER_REGISTERED    = QStringLiteral("userRegistered");

enum class Reset
{
  Force = 1<<0,
  Silent = 1<<1,
};

Q_DECLARE_FLAGS(ResetFlags, Reset)
Q_DECLARE_OPERATORS_FOR_FLAGS(ResetFlags)

using id_t = GroundControlPointsHelper::id_t;
using gcp_sptr = GroundControlPointsHelper::gcp_sptr;

//-----------------------------------------------------------------------------
struct GroundControlPoint
{
  kv::ground_control_point_sptr gcp;
  kv::track_sptr feature;
};

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
GroundControlPoint extractGroundControlPoint(QJsonObject const& f)
{
  // Check for geometry
  auto const& geom = f.value(TAG_GEOMETRY).toObject();
  if (geom.isEmpty())
  {
    qDebug() << "ignoring feature" << f
             << "with bad or missing geometry";
    return {};
  }

  // Check geometry type
  if (geom.value(TAG_TYPE).toString() != TAG_POINT)
  {
    // Non-point features are silently ignored
    return {};
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
    gcp = nullptr;
  }
  else
  {
    gcp->set_geo_loc_user_provided(props.value(TAG_USER_REGISTERED).toBool());
  }

  // Read feature track
  auto ft = kv::track::create();
  auto const& regs = props.value(TAG_REGISTRATIONS).toArray();
  for (auto const& riter : regs)
  {
    if (!riter.isObject())
    {
      continue;
    }

    auto const& reg = riter.toObject();
    auto const& frame = reg.value(TAG_FRAME);
    if (frame.isDouble())
    {
      auto const& loc = reg.value(TAG_LOCATION).toArray();
      if (loc.size() == 2 && isDoubleArray(loc))
      {
        auto const t = static_cast<kv::frame_id_t>(frame.toDouble());
        auto feature = std::make_shared<kv::feature_d>();
        feature->set_loc({loc[0].toDouble(), loc[1].toDouble()});

        ft->insert(
          std::make_shared<kv::feature_track_state>(t, std::move(feature)));
      }
    }
  }
  if (ft->empty())
  {
    ft = nullptr;
  }

  // Check if we read anything usable
  if (!gcp && !ft)
  {
    qDebug() << "ignoring point feature" << f
             << "with no valid location information";
    return {};
  }

  return {gcp, ft};
}

//-----------------------------------------------------------------------------
QJsonValue buildFeature(GroundControlPoint const& item)
{
  if (!item.gcp && !item.feature)
  {
    return {};
  }

  // Create geometry
  auto geom = QJsonObject{};
  auto props = QJsonObject{};
  geom.insert(TAG_TYPE, TAG_POINT);

  if (item.gcp)
  {
    // Get point and point's locations (scene, world/geodetic)
    auto const& gcp = *(item.gcp);
    auto const& sl = gcp.loc();
    auto const& wl = gcp.geo_loc();

    if (!wl.is_empty())
    {
      auto const& rwl = wl.location(kv::SRID::lat_lon_WGS84);
      geom.insert(TAG_COORDINATES, QJsonArray{rwl[0], rwl[1], gcp.elevation()});
    }

    // Fill in ground control point properties
    props.insert(TAG_NAME, qtString(gcp.name()));
    props.insert(TAG_LOCATION, QJsonArray{sl[0], sl[1], sl[2]});
    props.insert(TAG_USER_REGISTERED, gcp.is_geo_loc_user_provided());
  }

  if (item.feature)
  {
    auto regs = QJsonArray{};
    for (auto const& s : (*item.feature) | kv::as_feature_track)
    {
      auto const& l = s->feature->loc();

      auto reg = QJsonObject{};
      reg.insert(TAG_FRAME, static_cast<double>(s->frame()));
      reg.insert(TAG_LOCATION, QJsonArray{l[0], l[1]});

      regs.append(reg);
    }
    props.insert(TAG_REGISTRATIONS, regs);
  }

  // Create and return feature
  auto f = QJsonObject{};
  f.insert(TAG_TYPE, TAG_FEATURE);
  f.insert(TAG_GEOMETRY, geom);
  f.insert(TAG_PROPERTIES, props);

  return f;
}

//-----------------------------------------------------------------------------
template <typename Func>
void executeForHandle(
  id_t id, GroundControlPointsWidget* widget,
  std::map<id_t, vtkHandleWidget*> const& idToHandleMap, Func const& func)
{
  auto* const hp = qtGet(idToHandleMap, id);
  if (auto* const handleWidget = (hp ? hp->second : nullptr))
  {
    auto const handleId = widget->findHandleWidget(handleWidget);

    if (handleId < 0)
    {
      qWarning()
        << "Failed to find the VTK ID associated with the VTK handle"
        << handleWidget << " and the point with ID" << id;
    }
    else
    {
      func(handleId, handleWidget);
    }
  }
  else
  {
    qWarning() << "Failed to find the VTK handle associated with "
                  "the point with ID" << id;
  }
}

//-----------------------------------------------------------------------------
kv::feature_track_state_sptr getFeature(
  GroundControlPoint const& gcp, kv::frame_id_t frame)
{
  if (gcp.feature)
  {
    if (auto* const s = qtGet(*gcp.feature, frame))
    {
      return std::dynamic_pointer_cast<kv::feature_track_state>(*s);
    }
  }
  return nullptr;
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
  GroundControlPointsWidget* crpWidget = nullptr;

  id_t activeId = std::numeric_limits<id_t>::max();
  id_t nextId = 0;
  std::map<id_t, GroundControlPoint> groundControlPoints;
  std::map<id_t, vtkHandleWidget*> gcpIdToHandleMap;
  std::map<id_t, vtkHandleWidget*> crpIdToHandleMap;
  std::map<vtkHandleWidget*, id_t> gcpHandleToIdMap;
  std::map<vtkHandleWidget*, id_t> crpHandleToIdMap;

  void addPoint(id_t id, gcp_sptr const& point);

  void updatePoint(int handleId);
  void movePoint(int handleId, GroundControlPointsWidget* widget,
                 kv::vector_3d const& newPosition);

  void resetPoint(id_t gcpId, ResetFlags options = {});
  void resetPoint(gcp_sptr const& gcp, id_t gcpId, ResetFlags options = {});

  void removeGcp(int handleId, vtkHandleWidget* handleWidget);
  void removeCrp(int handleId, vtkHandleWidget* handleWidget);

  void updateActivePoint(
    int handleId, GroundControlPointsWidget* widget,
    std::map<vtkHandleWidget*, id_t> const& handleToIdMap);

  bool addCameraViewPoint();

  id_t addGroundControlPoint();
  id_t addRegistrationPoint();

private:
  QTE_DECLARE_PUBLIC_PTR(GroundControlPointsHelper)
  QTE_DECLARE_PUBLIC(GroundControlPointsHelper)
};

QTE_IMPLEMENT_D_FUNC(GroundControlPointsHelper)

//-----------------------------------------------------------------------------
void GroundControlPointsHelperPrivate::addPoint(id_t id, gcp_sptr const& point)
{
  // Add point to internal map
  this->groundControlPoints[id].gcp = point;

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
  auto const gcp = (gcpp ? gcpp->second.gcp : nullptr);

  if (!gcp)
  {
    qWarning() << "No ground control point with id" << gcpId;
    return;
  }

  this->resetPoint(gcp, gcpId, options);
}

//-----------------------------------------------------------------------------
void GroundControlPointsHelperPrivate::resetPoint(
  gcp_sptr const& gcp, id_t gcpId, ResetFlags options)
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
    auto const gcp = (gcpIter ? gcpIter->second.gcp : nullptr);

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
void GroundControlPointsHelperPrivate::removeGcp(
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

  QTE_Q();

  auto iter = this->groundControlPoints.find(gcpId);
  Q_ASSERT(iter != this->groundControlPoints.end());

  if (!iter->second.feature)
  {
    this->groundControlPoints.erase(iter);

    emit q->pointRemoved(gcpId);
    emit q->pointCountChanged(this->groundControlPoints.size());
  }
  else
  {
    iter->second.gcp = nullptr;

    emit q->pointChanged(gcpId);
  }
}

//-----------------------------------------------------------------------------
void GroundControlPointsHelperPrivate::removeCrp(
  int handleId, vtkHandleWidget* handleWidget)
{
  auto const& i = this->crpHandleToIdMap.find(handleWidget);
  if (i == this->crpHandleToIdMap.end())
  {
    qWarning() << "Failed to find the ID associated with the handle widget"
               << handleWidget << "with VTK ID" << handleId;
    return;
  }

  auto const gcpId = i->second;
  this->crpHandleToIdMap.erase(i);
  this->crpIdToHandleMap.erase(gcpId);

  QTE_Q();

  auto iter = this->groundControlPoints.find(gcpId);
  Q_ASSERT(iter != this->groundControlPoints.end());

  if (!iter->second.feature)
  {
    qWarning() << "Unable to remove feature track point "
                  "for ground control point" << gcpId
               << "as there is no feature track?";
    return;
  }

  auto const t = this->mainWindow->activeFrame();

  auto& track = *iter->second.feature;
  if (track.remove(t) && track.empty())
  {
    if (!iter->second.gcp)
    {
      this->groundControlPoints.erase(iter);

      emit q->pointRemoved(gcpId);
      emit q->pointCountChanged(this->groundControlPoints.size());

      return;
    }

    iter->second.feature.reset();
  }

  emit q->pointChanged(gcpId);
}

//-----------------------------------------------------------------------------
id_t GroundControlPointsHelperPrivate::addGroundControlPoint()
{
  auto id = this->activeId;
  auto const p = qtGet(this->groundControlPoints, id);

  // Are we adding a world location to an existing point or making new point?
  if (!p || p->second.gcp)
  {
    id = this->nextId++;
  }

  // Create the point and update the handle mapping
  auto const pt = this->worldWidget->activePoint();
  this->groundControlPoints[id].gcp =
    std::make_shared<kv::ground_control_point>(pt);
  vtkHandleWidget* const handle =
    this->worldWidget->handleWidget(this->worldWidget->activeHandle());

  this->gcpHandleToIdMap[handle] = id;
  this->gcpIdToHandleMap[id] = handle;

  this->resetPoint(id, Reset::Silent);

  // Return the identifier of the created point
  return id;
}

//-----------------------------------------------------------------------------
id_t GroundControlPointsHelperPrivate::addRegistrationPoint()
{
  auto id = this->activeId;
  auto const p = qtGet(this->groundControlPoints, id);
  auto const t = this->mainWindow->activeFrame();

  // Are we adding a world location to an existing point or making new point?
  if (!p || (p->second.feature && p->second.feature->contains(t)))
  {
    id = this->nextId++;
  }

  // Create the track, if necessary
  auto& track = this->groundControlPoints[id].feature;
  if (!track)
  {
    track = kv::track::create();
    track->set_id(id);
  }

  // Create the feature
  auto feature = std::make_shared<kv::feature_d>();
  auto const pt = this->crpWidget->activePoint();
  feature->set_loc({pt[0], pt[1]});

  // Update the feature track
  auto s = std::make_shared<kv::feature_track_state>(t, std::move(feature));
  track->insert(s);

  // Update the handle mapping
  vtkHandleWidget* const handle =
    this->crpWidget->handleWidget(this->crpWidget->activeHandle());

  this->crpHandleToIdMap[handle] = id;
  this->crpIdToHandleMap[id] = handle;

  // Return the identifier of the created point
  return id;
}

//-----------------------------------------------------------------------------
void GroundControlPointsHelperPrivate::updateActivePoint(
  int handleId, GroundControlPointsWidget* widget,
  std::map<vtkHandleWidget*, id_t> const& handleToIdMap)
{
  if (handleId >= 0)
  {
    QTE_Q();

    // Find the corresponding GCP ID
    auto* const handle = widget->handleWidget(handleId);
    if (auto* const iter = qtGet(handleToIdMap, handle))
    {
      emit q->activePointChanged(this->activeId = iter->second);
    }
    else
    {
      qWarning() << "Failed to find the ID associated with the handle widget"
                << handle << "with VTK ID" << handleId;
    }
  }
  else
  {
    this->activeId = std::numeric_limits<id_t>::max();
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
  d->crpWidget = d->mainWindow->cameraView()->registrationPointsWidget();

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
          this, &GroundControlPointsHelper::removeGcpByHandle);
  connect(d->cameraWidget, &GroundControlPointsWidget::pointDeleted,
          this, &GroundControlPointsHelper::removeGcpByHandle);
  connect(
    d->worldWidget, &GroundControlPointsWidget::activePointChanged, this,
    [d](int handleId){
      d->updateActivePoint(handleId, d->worldWidget, d->gcpHandleToIdMap);
    });
  connect(
    d->crpWidget, &GroundControlPointsWidget::activePointChanged, this,
    [d](int handleId){
      d->updateActivePoint(handleId, d->crpWidget, d->crpHandleToIdMap);
    });

  connect(d->crpWidget, &GroundControlPointsWidget::pointPlaced,
          this, &GroundControlPointsHelper::addRegistrationPoint);
  connect(d->crpWidget, &GroundControlPointsWidget::pointMoved,
          this, &GroundControlPointsHelper::moveRegistrationPoint);
  connect(d->crpWidget, &GroundControlPointsWidget::pointDeleted,
          this, &GroundControlPointsHelper::removeCrpByHandle);

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
void GroundControlPointsHelper::addRegistrationPoint()
{
  QTE_D();

  emit this->pointAdded(d->addRegistrationPoint());
  emit this->pointCountChanged(d->groundControlPoints.size());
}

//-----------------------------------------------------------------------------
void GroundControlPointsHelper::addCameraViewPoint()
{
  QTE_D();

  d->addCameraViewPoint();

  emit this->pointAdded(d->addGroundControlPoint());
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

  emit this->pointAdded(d->addGroundControlPoint());
  emit this->pointCountChanged(d->groundControlPoints.size());
}

//-----------------------------------------------------------------------------
void GroundControlPointsHelper::removeGcpByHandle(int handleId)
{
  QTE_D();

  auto* const handleWidget = d->worldWidget->handleWidget(handleId);
  if (!handleWidget)
  {
    qWarning() << "Failed to find the VTK handle widget with VTK ID"
               << handleId;
    return;
  }

  d->removeGcp(handleId, handleWidget);
}

//-----------------------------------------------------------------------------
void GroundControlPointsHelper::removeCrpByHandle(int handleId)
{
  QTE_D();

  auto* const handleWidget = d->crpWidget->handleWidget(handleId);
  if (!handleWidget)
  {
    qWarning() << "Failed to find the VTK handle widget with VTK ID"
               << handleId;
    return;
  }

  d->removeCrp(handleId, handleWidget);
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
void GroundControlPointsHelper::moveRegistrationPoint()
{
  QTE_D();

  auto const handleId = d->crpWidget->activeHandle();
  auto* const handle = d->crpWidget->handleWidget(handleId);

  if (auto pid = qtGet(d->crpHandleToIdMap, handle))
  {
    if (auto p = qtGet(d->groundControlPoints, pid->second))
    {
      auto const frame = d->mainWindow->activeFrame();
      if (auto const& s = getFeature(p->second, frame))
      {
        Q_ASSERT(s->feature); // Something went very wrong if this fails...

        auto const& loc = d->crpWidget->activePoint();
        auto const& feature =
          std::dynamic_pointer_cast<kv::feature_d>(s->feature);

        Q_ASSERT(feature); // Something went very wrong if this fails...
        feature->set_loc({loc[0], loc[1]});
      }
      else
      {
        qWarning() << "No feature for ground control point with id"
                   << pid->second << "at frame" << frame;
      }
    }
    else
    {
      qWarning() << "No ground control point with id" << pid->second;
    }
  }
  else
  {
    qWarning() << "Failed to find the ID associated with the handle widget"
               << handle << "with VTK ID" << handleId;
  }

}

//-----------------------------------------------------------------------------
void GroundControlPointsHelper::updateCameraViewPoints()
{
  QTE_D();

  // Update camera registration points in camera view
  auto const frame = d->mainWindow->activeFrame();
  with_expr (qtScopedBlockSignals{d->crpWidget})
  {
    d->crpHandleToIdMap.clear();
    d->crpIdToHandleMap.clear();
    auto activeHandle = decltype(d->crpWidget->activeHandle()){-1};

    auto widgetPointCount = d->crpWidget->numberOfPoints();
    auto pointsUsed = decltype(widgetPointCount){0};

    for (auto const& p : d->groundControlPoints)
    {
      if (auto const& s = getFeature(p.second, frame))
      {
        if (s->feature)
        {
          auto const& loc = s->feature->loc();
          if (pointsUsed >= widgetPointCount)
          {
            d->crpWidget->addPoint(loc[0], loc[1], 0.0);
          }
          else
          {
            d->crpWidget->movePoint(pointsUsed, loc[0], loc[1], 0.0);
          }

          auto const h = d->crpWidget->handleWidget(pointsUsed);
          d->crpHandleToIdMap.emplace(h, p.first);
          d->crpIdToHandleMap.emplace(p.first, h);

          if (p.first == d->activeId)
          {
            activeHandle = pointsUsed;
          }

          ++pointsUsed;
        }
      }
    }

    while (widgetPointCount > pointsUsed)
    {
      d->crpWidget->deletePoint(--widgetPointCount);
    }

    d->crpWidget->setActivePoint(activeHandle);
  }

  kwiver::arrows::vtk::vtkKwiverCamera* camera = d->mainWindow->activeCamera();
  if (!camera)
  {
    d->cameraWidget->clearPoints();
    return;
  }

  // Update ground control points in camera view
  with_expr (qtScopedBlockSignals{d->cameraWidget})
  {
    auto const activeHandle = d->worldWidget->activeHandle();

    auto const worldPointCount = d->worldWidget->numberOfPoints();
    auto cameraPointCount = d->cameraWidget->numberOfPoints();
    while (cameraPointCount > worldPointCount)
    {
      d->cameraWidget->deletePoint(--cameraPointCount);
    }

    for (auto const i : qtIndexRange(worldPointCount))
    {
      auto const worldPt = d->worldWidget->point(i);
      double cameraPt[2];
      camera->ProjectPoint(worldPt, cameraPt);
      if (i >= cameraPointCount)
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
      kwiver::vital::vector_3d loc = i.second.gcp->loc();
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
    d->resetPoint(i.second.gcp, i.first, Reset::Silent);
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
void GroundControlPointsHelper::removePoint(id_t id)
{
  QTE_D();

  auto* const gcp = qtGet(d->groundControlPoints, id);
  if (!gcp)
  {
    qWarning() << __func__ << "called with non-existing id" << id;
    return;
  }

  // Delete ground control point (if applicable)
  if (gcp->second.gcp)
  {
    executeForHandle(
      id, d->worldWidget, d->gcpIdToHandleMap,
      [d](int handleId, vtkHandleWidget* handleWidget)
      {
        d->worldWidget->deletePoint(handleId);
        d->cameraWidget->deletePoint(handleId);
        d->removeGcp(handleId, handleWidget);
      });
  }

  if (gcp->second.feature)
  {
    executeForHandle(
      id, d->crpWidget, d->crpIdToHandleMap,
      [d](int handleId, vtkHandleWidget* handleWidget)
      {
        d->crpWidget->deletePoint(handleId);
        d->removeCrp(handleId, handleWidget);
      });
  }
}

//-----------------------------------------------------------------------------
void GroundControlPointsHelper::setActivePoint(id_t id)
{
  QTE_D();

  auto* const gcp = qtGet(d->groundControlPoints, id);
  if (!gcp)
  {
    if (id == std::numeric_limits<id_t>::max())
    {
      d->activeId = id;
      d->worldWidget->setActivePoint(INVALID_HANDLE);
      d->cameraWidget->setActivePoint(INVALID_HANDLE);
      d->crpWidget->setActivePoint(INVALID_HANDLE);
    }
    else
    {
      qWarning() << __func__ << "called with non-existing id" << id;
    }
    return;
  }

  // Set active ground control point
  if (gcp->second.gcp)
  {
    executeForHandle(
      id, d->worldWidget, d->gcpIdToHandleMap,
      [d](int handleId, vtkHandleWidget*)
      {
        d->worldWidget->setActivePoint(handleId);
        d->cameraWidget->setActivePoint(handleId);
      });
  }
  else
  {
    d->worldWidget->setActivePoint(INVALID_HANDLE);
    d->cameraWidget->setActivePoint(INVALID_HANDLE);
  }

  // Set active camera registration point
  if (gcp->second.feature)
  {
    executeForHandle(
      id, d->crpWidget, d->crpIdToHandleMap,
      [d](int handleId, vtkHandleWidget*)
      {
        d->crpWidget->setActivePoint(handleId);
      });
  }
  else
  {
    d->crpWidget->setActivePoint(INVALID_HANDLE);
  }

  // Update the active ID (last, so that our own changes to the widgets don't
  // cause their updates to invalidate the active ID in case we had to clear a
  // widget's active point)
  d->activeId = id;
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
    d->crpWidget->clearPoints();
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
bool GroundControlPointsHelper::isEmpty() const
{
  QTE_D();

  return d->groundControlPoints.empty();
}

//-----------------------------------------------------------------------------
std::vector<id_t> GroundControlPointsHelper::identifiers() const
{
  QTE_D();

  auto out = std::vector<id_t>{};
  out.reserve(d->groundControlPoints.size());

  for (auto const& iter : d->groundControlPoints)
  {
    out.emplace_back(iter.first);
  }

  return out;
}

//-----------------------------------------------------------------------------
std::vector<gcp_sptr> GroundControlPointsHelper::groundControlPoints() const
{
  QTE_D();

  auto out = std::vector<gcp_sptr>{};
  out.reserve(d->groundControlPoints.size());

  for (auto const& iter : d->groundControlPoints)
  {
    if (iter.second.gcp)
    {
      out.emplace_back(iter.second.gcp);
    }
  }

  return out;
}

//-----------------------------------------------------------------------------
kv::feature_track_set_sptr
GroundControlPointsHelper::registrationTracks() const
{
  QTE_D();

  auto out = std::make_shared<kv::feature_track_set>();

  for (auto const& iter : d->groundControlPoints)
  {
    if (iter.second.feature)
    {
      out->insert(iter.second.feature);
    }
  }

  return out;
}

//-----------------------------------------------------------------------------
kv::landmark_map_sptr GroundControlPointsHelper::registrationLandmarks() const
{
  QTE_D();

  auto landmarks = kv::landmark_map::map_landmark_t{};

  for (auto const& iter : d->groundControlPoints)
  {
    if (iter.second.gcp)
    {
      auto lm = std::make_shared<kv::landmark_d>(iter.second.gcp->loc());
      landmarks.emplace(iter.first, lm);
    }
  }

  return std::make_shared<kv::simple_landmark_map>(landmarks);
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
  auto const activeFrame = d->mainWindow->activeFrame();
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
      auto const& gcp = extractGroundControlPoint(fo);
      if (gcp.gcp)
      {
        d->addPoint(d->nextId, gcp.gcp);
      }
      if (gcp.feature)
      {
        gcp.feature->set_id(d->nextId);
        d->groundControlPoints[d->nextId].feature = gcp.feature;

        auto const& si = gcp.feature->find(activeFrame);
        if (si != gcp.feature->end())
        {
          auto const& s = kv::feature_track_state::downcast(*si);
          auto const& loc = s->feature->loc();
          auto const k = d->crpWidget->addPoint(loc[0], loc[1], 0.0);
          auto const h = d->crpWidget->handleWidget(k);

          d->crpHandleToIdMap.emplace(h, d->nextId);
          d->crpIdToHandleMap.emplace(d->nextId, h);
        }
      }
      if (gcp.gcp || gcp.feature)
      {
        ++d->nextId;
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

  // Build feature array
  auto features = QJsonArray{};
  auto errors = QStringList{};
  auto framesUsed = QSet<kv::frame_id_t>{};
  for (auto const& gcpi : d->groundControlPoints)
  {
    // Extract set of cameras used
    if (gcpi.second.feature)
    {
      for (auto const& s : (*gcpi.second.feature))
      {
        framesUsed.insert(s->frame());
      }
    }

    // Create feature
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

  // Build frame name map
  auto frames = QJsonArray{};
  for (auto fi : framesUsed)
  {
    auto frame = QJsonObject{};
    frame.insert(TAG_FRAME, static_cast<double>(fi));
    frame.insert(TAG_NAME, d->mainWindow->frameName(fi));
    frames.append(frame);
  }

  auto props = QJsonObject{};
  props.insert(TAG_FRAMES, frames);

  // Build feature collection
  auto root = QJsonObject{};
  root.insert(TAG_TYPE, TAG_FEATURECOLLECTION);
  root.insert(TAG_FEATURES, features);
  root.insert(TAG_PROPERTIES, props);

  // Write JSON
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
gcp_sptr GroundControlPointsHelper::groundControlPoint(id_t pointId)
{
  QTE_D();
  auto it = d->groundControlPoints.find(pointId);
  return it == d->groundControlPoints.end() ? nullptr : it->second.gcp;
}
