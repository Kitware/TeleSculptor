// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#ifndef TELESCULPTOR_CAMERAOPTIONS_H_
#define TELESCULPTOR_CAMERAOPTIONS_H_

#include <qtGlobal.h>

#include <QWidget>

class vtkMaptkCameraRepresentation;

class CameraOptionsPrivate;

class CameraOptions : public QWidget
{
  Q_OBJECT

public:
  explicit CameraOptions(vtkMaptkCameraRepresentation*,
                         QWidget* parent = nullptr,
                         Qt::WindowFlags flags = {});
  ~CameraOptions() override;

signals:
  void modified();

public slots:
  void setCamerasVisible(bool);

  void setBaseCameraScale(double);

protected slots:
  void updateScale();
  void updateInactiveDisplayOptions();

  void setPathVisible(bool);
  void setInactiveVisible(bool);

private:
  QTE_DECLARE_PRIVATE_RPTR(CameraOptions)
  QTE_DECLARE_PRIVATE(CameraOptions)

  QTE_DISABLE_COPY(CameraOptions)
};

#endif
