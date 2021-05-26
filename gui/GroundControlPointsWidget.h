// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

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
  int addDisplayPoint(double pt[3]);
  int addDisplayPoint(double x, double y, double z);
  int addDisplayPoint(kwiver::vital::vector_3d pt);
  int addPoint(double const pt[3]);
  int addPoint(double x, double y, double z);
  int addPoint(kwiver::vital::vector_3d pt);
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
