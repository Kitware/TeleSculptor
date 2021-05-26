// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#ifndef TELESCULPTOR_RULERHELPER_H_
#define TELESCULPTOR_RULERHELPER_H_

// qtExtensions includes
#include <qtGlobal.h>

// Qt declarations
#include <QColor>
#include <QObject>

// Forward declarations
class RulerHelperPrivate;
class RulerWidget;

class RulerHelper : public QObject
{
  Q_OBJECT

public:
  RulerHelper(QObject* parent = nullptr);
  ~RulerHelper();

  // Update the camera view ruler from the world view
  void updateCameraViewRuler();

  RulerWidget* worldWidget();
  RulerWidget* cameraWidget();

public slots:
  void enableWidgets(bool);
  void resetRuler();
  void setRulerTickDistance(double scale);
  void setRulerColor(const QColor& rgb);

protected slots:
  void addWorldViewPoint(int pId);
  void addCameraViewPoint(int pId);

  void moveCameraViewPoint(int pId);
  void moveWorldViewPoint(int pId);

private:
  QTE_DECLARE_PRIVATE_RPTR(RulerHelper)
  QTE_DECLARE_PRIVATE(RulerHelper)

  QTE_DISABLE_COPY(RulerHelper)
};

#endif
