// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#ifndef TELESCULPTOR_DATAFILTEROPTIONS_H_
#define TELESCULPTOR_DATAFILTEROPTIONS_H_

#include <qtGlobal.h>

#include <QWidget>

class DataFilterOptionsPrivate;

class DataFilterOptions : public QWidget
{
  Q_OBJECT

public:
  enum ActiveFilter
  {
    Minimum = 0x1,
    Maximum = 0x2,
  };
  Q_DECLARE_FLAGS(ActiveFilters, ActiveFilter)

  explicit DataFilterOptions(QString const& settingsGroup,
                             QWidget* parent = nullptr,
                             Qt::WindowFlags flags = {});
  ~DataFilterOptions() override;

  double minimum() const;
  double maximum() const;
  ActiveFilters activeFilters() const;

  QString iconText() const;

public slots:
  void setAvailableRange(double lower, double upper);

signals:
  void modified();

protected slots:
  void resetRange();

  void updateMinimum();
  void updateMaximum();

private:
  QTE_DECLARE_PRIVATE_RPTR(DataFilterOptions)
  QTE_DECLARE_PRIVATE(DataFilterOptions)

  QTE_DISABLE_COPY(DataFilterOptions)
};

Q_DECLARE_OPERATORS_FOR_FLAGS(DataFilterOptions::ActiveFilters)

#endif
