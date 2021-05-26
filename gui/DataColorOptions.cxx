// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#include "DataColorOptions.h"

#include "ui_DataColorOptions.h"

#include "vtkMaptkScalarsToGradient.h"

#include <vtkNew.h>

#include <qtGradient.h>
#include <qtScopedValueChange.h>
#include <qtUiState.h>
#include <qtUiStateItem.h>

#include <cmath>

QTE_IMPLEMENT_D_FUNC(DataColorOptions)

//-----------------------------------------------------------------------------
class DataColorOptionsPrivate
{
public:
  Ui::DataColorOptions UI;
  qtUiState uiState;

  double autoLower;
  double autoUpper;

  vtkNew<vtkMaptkScalarsToGradient> scalarsToGradient;
};

//-----------------------------------------------------------------------------
DataColorOptions::DataColorOptions(
  QString const& settingsGroup, QWidget* parent, Qt::WindowFlags flags)
  : QWidget(parent, flags), d_ptr(new DataColorOptionsPrivate)
{
  QTE_D();

  // Set up UI
  d->UI.setupUi(this);

  d->uiState.setCurrentGroup(settingsGroup + "/DataColor");

  d->uiState.mapChecked("AutomaticMinimum", d->UI.autoMinimum);
  d->uiState.mapChecked("AutomaticMaximum", d->UI.autoMaximum);
  d->uiState.mapValue("Minimum", d->UI.minimum);
  d->uiState.mapValue("Maximum", d->UI.maximum);

  auto const gradientItem = new qtUiState::Item<int, QComboBox>(
    d->UI.gradient, &QComboBox::currentIndex, &QComboBox::setCurrentIndex);
  d->uiState.map("Colors", gradientItem);

  d->uiState.restore();

  d->scalarsToGradient->SetGradient(d->UI.gradient->currentGradient());

  connect(d->UI.minimum, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &DataColorOptions::updateMinimum);
  connect(d->UI.maximum, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &DataColorOptions::updateMaximum);
  connect(d->UI.autoMinimum, &QAbstractButton::toggled,
          this, &DataColorOptions::updateMinimum);
  connect(d->UI.autoMaximum, &QAbstractButton::toggled,
          this, &DataColorOptions::updateMaximum);

  connect(d->UI.gradient, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &DataColorOptions::setGradient);
}

//-----------------------------------------------------------------------------
DataColorOptions::~DataColorOptions()
{
  QTE_D();
  d->uiState.save();
}

//-----------------------------------------------------------------------------
QIcon DataColorOptions::icon() const
{
  QTE_D();
  return d->UI.gradient->itemIcon(d->UI.gradient->currentIndex());
}

//-----------------------------------------------------------------------------
vtkScalarsToColors* DataColorOptions::scalarsToColors() const
{
  QTE_D();
  return d->scalarsToGradient;
}

//-----------------------------------------------------------------------------
double DataColorOptions::minimum() const
{
  QTE_D();
  return d->UI.minimum->value();
}

//-----------------------------------------------------------------------------
double DataColorOptions::maximum() const
{
  QTE_D();
  return d->UI.maximum->value();
}

//-----------------------------------------------------------------------------
void DataColorOptions::setAvailableRange(
  double lower, double upper, double autoLower, double autoUpper)
{
  QTE_D();

  qtScopedBlockSignals bl{d->UI.minimum};
  qtScopedBlockSignals bu{d->UI.maximum};

  d->UI.minimum->setRange(lower, upper);
  d->UI.maximum->setRange(lower, upper);

  auto const scale = std::floor(std::log10(std::abs(upper - lower)));
  auto const step = std::pow(10.0, scale - 1.0);

  d->UI.minimum->setSingleStep(step);
  d->UI.maximum->setSingleStep(step);

  d->autoLower = qMax(lower, autoLower);
  d->autoUpper = qMin(upper, autoUpper);

  this->updateMinimum();
  this->updateMaximum();
}

//-----------------------------------------------------------------------------
void DataColorOptions::setGradient(int which)
{
  QTE_D();

  d->UI.gradient->setCurrentIndex(which);
  d->scalarsToGradient->SetGradient(d->UI.gradient->currentGradient());

  emit this->modified();
  emit this->iconChanged(this->icon());
}

//-----------------------------------------------------------------------------
void DataColorOptions::updateMinimum()
{
  QTE_D();

  if (d->UI.autoMinimum->isChecked())
  {
    auto const softLimit =
      (d->UI.autoMaximum->isChecked() ? +qInf() : d->UI.maximum->value());
    d->UI.minimum->setValue(qMin(softLimit, d->autoLower));
  }

  auto const limit = d->UI.minimum->value();
  if (d->UI.maximum->value() < limit)
  {
    d->UI.maximum->setValue(limit);
  }

  d->scalarsToGradient->SetRange(d->UI.minimum->value(),
                                 d->UI.maximum->value());

  emit this->modified();
}

//-----------------------------------------------------------------------------
void DataColorOptions::updateMaximum()
{
  QTE_D();

  if (d->UI.autoMaximum->isChecked())
  {
    auto const softLimit =
      (d->UI.autoMinimum->isChecked() ? -qInf() : d->UI.minimum->value());
    d->UI.maximum->setValue(qMax(softLimit, d->autoUpper));
  }

  auto const limit = d->UI.maximum->value();
  if (d->UI.minimum->value() > limit)
  {
    d->UI.minimum->setValue(limit);
  }

  d->scalarsToGradient->SetRange(d->UI.minimum->value(),
                                 d->UI.maximum->value());

  emit this->modified();
}
