// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#ifndef TELESCULPTOR_DEPTHMAPOPTIONS_H_
#define TELESCULPTOR_DEPTHMAPOPTIONS_H_

#include <qtGlobal.h>

#include <QWidget>

class vtkProp3D;

class DepthMapOptionsPrivate;

class DepthMapOptions : public QWidget
{
  Q_OBJECT

public:
  enum DisplayMode
  {
    Points,
    Surfaces,
  };

public:
  explicit DepthMapOptions(QString const& settingsGroup,
                           QWidget* parent = nullptr,
                           Qt::WindowFlags flags = {});
  ~DepthMapOptions() override;

  DisplayMode displayMode() const;

  double weightMinimum() const;
  double weightMaximum() const;
  double uncertaintyMinimum() const;
  double uncertaintyMaximum() const;

  void initializeFilters(double bcMin, double bcMax,
                         double urMin, double urMax);

  bool isFilterPersistent() const;
  bool isFilterEnabled() const;

signals:
  void displayModeChanged();
  void thresholdsChanged(bool);

protected slots:
  void thresholdsChanged();

private:
  QTE_DECLARE_PRIVATE_RPTR(DepthMapOptions)
  QTE_DECLARE_PRIVATE(DepthMapOptions)

  QTE_DISABLE_COPY(DepthMapOptions)
};

#endif
