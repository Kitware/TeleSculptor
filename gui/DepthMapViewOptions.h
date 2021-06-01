// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#ifndef TELESCULPTOR_DEPTHMAPVIEWOPTIONS_H_
#define TELESCULPTOR_DEPTHMAPVIEWOPTIONS_H_

#include <qtGlobal.h>

#include <QButtonGroup>
#include <QVBoxLayout>
#include <QWidget>

class vtkActor;
class vtkPointData;

class DepthMapViewOptionsPrivate;

class DepthMapViewOptions : public QWidget
{
  Q_OBJECT

public:
  explicit DepthMapViewOptions(QString const& settingsGroup,
                               QWidget* parent = nullptr,
                               Qt::WindowFlags flags = {});
  ~DepthMapViewOptions() override;

  void setActor(vtkActor*);

signals:
  void modified();

public slots:
  void updateRanges(vtkPointData*);
  void updateActor();

protected slots:
  void setDepthIcon(QIcon const&);
  void setWeightIcon(QIcon const&);
  void setUncertaintyIcon(QIcon const&);

private:
  QTE_DECLARE_PRIVATE_RPTR(DepthMapViewOptions)
  QTE_DECLARE_PRIVATE(DepthMapViewOptions)

  QTE_DISABLE_COPY(DepthMapViewOptions)
};

#endif
