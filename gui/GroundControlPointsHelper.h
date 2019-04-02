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

#ifndef TELESCULPTOR_GROUNDCONTROLPOINTSHELPER_H_
#define TELESCULPTOR_GROUNDCONTROLPOINTSHELPER_H_

// maptk includes
#include <maptk/ground_control_point.h>

// qtExtensions includes
#include <qtGlobal.h>

// Qt declarations
#include <QObject>

// Forward declarations
class GroundControlPointsHelperPrivate;

class GroundControlPointsHelper : public QObject
{
  Q_OBJECT

public:
  GroundControlPointsHelper(QObject* parent = nullptr);
  ~GroundControlPointsHelper();

  void updateCameraViewPoints();

  // Set ground control points
  void setGroundControlPoints(kwiver::vital::ground_control_point_map const&);
  // Get access to the ground control points
  kwiver::vital::ground_control_point_map::ground_control_point_map_t const&
  groundControlPoints() const;

  // Get access to a single ground control point
  kwiver::vital::ground_control_point_sptr groundControlPoint(
    kwiver::vital::ground_control_point_id_t pointId);

  bool readGroundControlPoints(QString const& path);

  bool writeGroundControlPoints(
    QString const& path, QWidget* dialogParent) const;

public slots:
  void enableWidgets(bool);

  void recomputePoints();

  void resetPoint(kwiver::vital::ground_control_point_id_t);
  void removePoint(kwiver::vital::ground_control_point_id_t);

  void setActivePoint(kwiver::vital::ground_control_point_id_t);

  void applySimilarityTransform();

signals:
  void pointsReloaded();
  void pointsRecomputed();
  void pointCountChanged(size_t);
  void pointAdded(kwiver::vital::ground_control_point_id_t);
  void pointRemoved(kwiver::vital::ground_control_point_id_t);
  void pointChanged(kwiver::vital::ground_control_point_id_t);

  void activePointChanged(kwiver::vital::ground_control_point_id_t);

protected slots:
  void addCameraViewPoint();
  void addWorldViewPoint();

  void removePointByHandle(int handleId);

  void moveCameraViewPoint();
  void moveWorldViewPoint();

private:
  QTE_DECLARE_PRIVATE_RPTR(GroundControlPointsHelper)
  QTE_DECLARE_PRIVATE(GroundControlPointsHelper)

  QTE_DISABLE_COPY(GroundControlPointsHelper)
};

#endif
