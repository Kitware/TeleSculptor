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

#ifndef TELESCULPTOR_GROUNDCONTROLPOINTSWIDGET_H_
#define TELESCULPTOR_GROUNDCONTROLPOINTSWIDGET_H_

#include <vital/types/vector.h>

#include <qtGlobal.h>

#include <QColor>
#include <QObject>

#include <list>

// Forward declarations
class GroundControlPointsWidgetPrivate;
class vtkAbstractWidget;
class vtkHandleWidget;
class vtkMatrix4x4;
class vtkObject;
class vtkPointPlacer;
class vtkRenderWindowInteractor;
class vtkRenderer;

class GroundControlPointsWidget : public QObject
{
  Q_OBJECT

public:
  GroundControlPointsWidget(QObject* parent = nullptr);
  ~GroundControlPointsWidget();

  void setInteractor(vtkRenderWindowInteractor* iren);

  void setPointPlacer(vtkPointPlacer* placer);

  // Get the renderer
  vtkRenderer* renderer();

  // Get active handle and point coordintes
  int activeHandle() const;
  kwiver::vital::vector_3d activePoint();
  kwiver::vital::vector_3d point(int handle) const;

  // Get access to the underlying handle widget
  vtkHandleWidget* handleWidget(int handleId) const;

  // Get ID of underlying handle widget
  int findHandleWidget(vtkHandleWidget* handle) const;

  // Add a point
  void addDisplayPoint(double pt[3]);
  void addDisplayPoint(double x, double y, double z);
  void addDisplayPoint(kwiver::vital::vector_3d pt);
  void addPoint(double const pt[3]);
  void addPoint(double x, double y, double z);
  void addPoint(kwiver::vital::vector_3d pt);
  void movePoint(int handleId, double x, double y, double z);
  void clearPoints();

  // If set, transform each point before adding
  // Note that this just affects new points. Perhaps, this can be extended to
  // transform each existing point in the future.
  void setTransformMatrix(vtkMatrix4x4*);
  vtkMatrix4x4* transformMatrix() const;

  // Get access to the points stored by the widget
  int numberOfPoints() const;

  // Render the widget
  void render();

  /// Enable/disable the internal seed widget
  void enableWidget(bool enable);

signals:
  void pointPlaced();
  void pointMoved();
  void pointDeleted(int id);
  void activePointChanged(int id);

public slots:
  void setColor(QColor);
  void setSelectedColor(QColor);

  /// Delete the point and remove the widget handle
  void deletePoint(int handleId);
  // Set active handle
  void setActivePoint(int id);

protected slots:
  // void addInternalPoint();
  void movePointEvent();
  void pointDeletedCallback(vtkObject*, unsigned long, void*, void*);
  void activeHandleChangedCallback(vtkObject*, unsigned long, void*, void*);

private:
  QTE_DECLARE_PRIVATE_RPTR(GroundControlPointsWidget)
  QTE_DECLARE_PRIVATE(GroundControlPointsWidget)

  QTE_DISABLE_COPY(GroundControlPointsWidget)
};

#endif
