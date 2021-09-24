// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#ifndef TELESCULPTOR_DEPTHMAPVIEW_H_
#define TELESCULPTOR_DEPTHMAPVIEW_H_

#include <qtGlobal.h>

#include <QWidget>

class DepthMapViewPrivate;
class vtkMaptkImageDataGeometryFilter;

class DepthMapView : public QWidget
{
  Q_OBJECT

public:
  explicit DepthMapView(QWidget* parent = nullptr, Qt::WindowFlags flags = {});
  ~DepthMapView() override;

  void enableAntiAliasing(bool enable);
public slots:

  void setValidDepthInput(bool);

  void updateView(bool);

  void updateThresholds();

  void setBackgroundColor(QColor const&);

  void setDepthGeometryFilter(vtkMaptkImageDataGeometryFilter*);

  void resetView();

  void increasePointSize();

  void decreasePointSize();

  void render();

private:
  QTE_DECLARE_PRIVATE_RPTR(DepthMapView)
  QTE_DECLARE_PRIVATE(DepthMapView)

  QTE_DISABLE_COPY(DepthMapView)
};

#endif
