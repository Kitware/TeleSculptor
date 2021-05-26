// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#ifndef TELESCULPTOR_GROUNDCONTROLPOINTSVIEW_H_
#define TELESCULPTOR_GROUNDCONTROLPOINTSVIEW_H_

#include <qtGlobal.h>

#include <QWidget>

class GroundControlPointsHelper;

class GroundControlPointsViewPrivate;

class GroundControlPointsView : public QWidget
{
  Q_OBJECT

public:
  explicit GroundControlPointsView(
    QWidget* parent = nullptr, Qt::WindowFlags flags = {});
  ~GroundControlPointsView();

  void setHelper(GroundControlPointsHelper*);

signals:
  void cameraRequested(qint64);

public slots:
  void setActiveCamera(qint64 id);

protected:
  void changeEvent(QEvent* e) override;
  void showEvent(QShowEvent* e) override;

private:
  QTE_DECLARE_PRIVATE_RPTR(GroundControlPointsView)
  QTE_DECLARE_PRIVATE(GroundControlPointsView)

  QTE_DISABLE_COPY(GroundControlPointsView)
};

#endif
