// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#ifndef TELESCULPTOR_DEPTHMAPFILTEROPTIONS_H_
#define TELESCULPTOR_DEPTHMAPFILTEROPTIONS_H_

#include <qtGlobal.h>

#include <QWidget>

class DepthMapFilterOptionsPrivate;

class DepthMapFilterOptions : public QWidget
{
  Q_OBJECT

public:
  explicit DepthMapFilterOptions(QString const& settingsGroup,
                                 QWidget* parent = nullptr,
                                 Qt::WindowFlags flags = {});
  ~DepthMapFilterOptions() override;

  double weightMinimum() const;
  double weightMaximum() const;
  double uncertaintyMinimum() const;
  double uncertaintyMaximum() const;

  void initializeFilters(double wMin, double wMax,
                         double uMin, double uMax);

  bool isFilterPersistent() const;

signals:
  void filtersChanged();

public slots:
  void updateWeightMinimum();
  void updateWeightMaximum();
  void updateUncertaintyMinimum();
  void updateUncertaintyMaximum();

  void resetFilters();

private:
  QTE_DECLARE_PRIVATE_RPTR(DepthMapFilterOptions)
  QTE_DECLARE_PRIVATE(DepthMapFilterOptions)

  QTE_DISABLE_COPY(DepthMapFilterOptions)
};

#endif
