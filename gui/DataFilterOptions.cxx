// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#include "DataFilterOptions.h"

#include "ui_DataFilterOptions.h"

#include <qtScopedValueChange.h>
#include <qtUiState.h>

QTE_IMPLEMENT_D_FUNC(DataFilterOptions)

//-----------------------------------------------------------------------------
class DataFilterOptionsPrivate
{
public:
  DataFilterOptionsPrivate() : minimum{-9999.99}, maximum{+9999.99} {}

  Ui::DataFilterOptions UI;
  qtUiState uiState;

  double minimum;
  double maximum;
};

//-----------------------------------------------------------------------------
DataFilterOptions::DataFilterOptions(
  QString const& settingsGroup, QWidget* parent, Qt::WindowFlags flags)
  : QWidget(parent, flags), d_ptr(new DataFilterOptionsPrivate)
{
  QTE_D();

  // Set up UI
  d->UI.setupUi(this);

  d->uiState.setCurrentGroup(settingsGroup + "/DataFilter");

  d->uiState.mapChecked("UseMinimum", d->UI.useMinimum);
  d->uiState.mapChecked("UseMaximum", d->UI.useMaximum);
  d->uiState.mapValue("Minimum", d->UI.minimum);
  d->uiState.mapValue("Maximum", d->UI.maximum);

  d->uiState.restore();

  connect(d->UI.minimum, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &DataFilterOptions::updateMinimum);
  connect(d->UI.maximum, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &DataFilterOptions::updateMaximum);
  connect(d->UI.useMinimum, &QAbstractButton::toggled,
          this, &DataFilterOptions::modified);
  connect(d->UI.useMaximum, &QAbstractButton::toggled,
          this, &DataFilterOptions::modified);

  connect(d->UI.reset, &QAbstractButton::clicked,
          this, &DataFilterOptions::resetRange);
}

//-----------------------------------------------------------------------------
DataFilterOptions::~DataFilterOptions()
{
  QTE_D();
  d->uiState.save();
}

//-----------------------------------------------------------------------------
double DataFilterOptions::minimum() const
{
  QTE_D();
  return qMax(d->minimum, d->UI.minimum->value());
}

//-----------------------------------------------------------------------------
double DataFilterOptions::maximum() const
{
  QTE_D();
  return qMin(d->maximum, d->UI.maximum->value());
}

//-----------------------------------------------------------------------------
DataFilterOptions::ActiveFilters DataFilterOptions::activeFilters() const
{
  QTE_D();

  auto result = ActiveFilters{};

  if (d->UI.useMinimum->isChecked())
  {
    result |= Minimum;
  }
  if (d->UI.useMaximum->isChecked())
  {
    result |= Maximum;
  }

  return result;
}

//-----------------------------------------------------------------------------
QString DataFilterOptions::iconText() const
{
  QTE_D();

  auto const lf = d->UI.useMinimum->isChecked();
  auto const hf = d->UI.useMaximum->isChecked();

  if (lf && hf)
  {
    return "(range)";
  }
  else if (lf)
  {
    return QString::fromUtf8("\xe2\x89\xa5 %1").arg(d->UI.minimum->value());
  }
  else if (hf)
  {
    return QString::fromUtf8("\xe2\x89\xa4 %1").arg(d->UI.maximum->value());
  }

  return "(none)";
}

//-----------------------------------------------------------------------------
void DataFilterOptions::setAvailableRange(double lower, double upper)
{
  QTE_D();

  d->minimum = lower;
  d->maximum = upper;

  qtScopedBlockSignals bl{d->UI.minimum};
  qtScopedBlockSignals bu{d->UI.maximum};

  this->updateMinimum();
  this->updateMaximum();
}

//-----------------------------------------------------------------------------
void DataFilterOptions::resetRange()
{
  QTE_D();

  d->UI.minimum->setRange(d->minimum, d->maximum);
  d->UI.maximum->setRange(d->minimum, d->maximum);
  d->UI.minimum->setValue(d->minimum);
  d->UI.maximum->setValue(d->maximum);
}

//-----------------------------------------------------------------------------
void DataFilterOptions::updateMinimum()
{
  QTE_D();

  auto const value = d->UI.minimum->value();
  d->UI.minimum->setRange(qMin(d->minimum, value), qMax(d->maximum, value));

  auto const limit = d->UI.minimum->value();
  if (d->UI.maximum->value() < limit)
  {
    d->UI.maximum->setValue(limit);
  }

  emit this->modified();
}

//-----------------------------------------------------------------------------
void DataFilterOptions::updateMaximum()
{
  QTE_D();

  auto const value = d->UI.maximum->value();
  d->UI.maximum->setRange(qMin(d->minimum, value), qMax(d->maximum, value));

  auto const limit = d->UI.maximum->value();
  if (d->UI.minimum->value() > limit)
  {
    d->UI.minimum->setValue(limit);
  }

  emit this->modified();
}
