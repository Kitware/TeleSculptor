// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#ifndef TELESCULPTOR_POINTOPTIONS_H_
#define TELESCULPTOR_POINTOPTIONS_H_

#include <qtGlobal.h>

#include <QWidget>

class vtkActor;
class vtkMapper;

struct FieldInformation;

class PointOptionsPrivate;

class PointOptions : public QWidget
{
  Q_OBJECT

public:
  explicit PointOptions(QString const& settingsGroup,
                        QWidget* parent = nullptr,
                        Qt::WindowFlags flags = {});
  ~PointOptions() override;

  void addActor(vtkActor*);
  void addMapper(vtkMapper*);

  void setDefaultColor(QColor const&);

public slots:
  void setTrueColorAvailable(bool);
  void setDataFields(QHash<QString, FieldInformation> const&);

signals:
  void modified();

protected slots:
  void setSize(int);
  void setColorMode(int);

  void setDataColorIcon(QIcon const&);

  void updateActiveDataField();
  void updateFilters();

private:
  QTE_DECLARE_PRIVATE_RPTR(PointOptions)
  QTE_DECLARE_PRIVATE(PointOptions)

  QTE_DISABLE_COPY(PointOptions)
};

#endif
