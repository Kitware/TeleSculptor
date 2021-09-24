// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#ifndef TELESCULPTOR_RULERWIDGET_H_
#define TELESCULPTOR_RULERWIDGET_H_

// qtExtensions includes
#include <qtGlobal.h>

// kwiver includes
#include <vital/types/vector.h>

// Qt includes
#include <QObject>

// STL includes
#include <list>

// Forward declarations
class RulerWidgetPrivate;
class vtkAbstractWidget;
class vtkHandleWidget;
class vtkMatrix4x4;
class vtkObject;
class vtkPointPlacer;
class vtkRenderWindowInteractor;
class vtkRenderer;

class RulerWidget : public QObject
{
  Q_OBJECT

public:
  RulerWidget(QObject* parent = nullptr);
  ~RulerWidget();

  void setInteractor(vtkRenderWindowInteractor* iren);

  void setPointPlacer(vtkPointPlacer* placer);

  // Get the renderer
  vtkRenderer* renderer();

  // If set, transform each point before adding
  // Note that this just affects new points. Perhaps, this can be extended to
  // transform each existing point in the future.
  void setTransformMatrix(vtkMatrix4x4*);
  vtkMatrix4x4* transformMatrix() const;

  // Render the widget
  void render();

  /// Enable/disable the internal seed widget
  void enableWidget(bool enable);

  // Set/Get the handle positions
  void setPoint1WorldPosition(double*);
  void setPoint1WorldPosition(double, double, double);
  kwiver::vital::vector_3d point1WorldPosition();
  void setPoint2WorldPosition(double*);
  void setPoint2WorldPosition(double, double, double);
  kwiver::vital::vector_3d point2WorldPosition();

  // Set/get whether the distance label should be computed on the widget
  void setComputeDistance(bool compute);

  // Set/Get the distance measurement
  void setDistance(double distance);
  double distance() const;

  // Get whether the ruler is placed
  bool isRulerPlaced() const;

  // Remove the ruler from the view
  void removeRuler();

  // Set/Get ruler scale
  void setRulerTickDistance(double scale);
  double rulerTickDistance() const;

  // Set/Get ruler color
  void setRulerColor(const QColor& rgb);
  QColor rulerColor() const;

signals:
  void pointPlaced(int pointId);
  void pointMoved(int pointId);
  void rulerPlaced(bool);

protected slots:
  void placePoint(vtkObject*, unsigned long, void*, void*);
  void movePointEvent(vtkObject*, unsigned long, void*, void*);

private:
  QTE_DECLARE_PRIVATE_RPTR(RulerWidget)
  QTE_DECLARE_PRIVATE(RulerWidget)

  QTE_DISABLE_COPY(RulerWidget)
};

#endif
