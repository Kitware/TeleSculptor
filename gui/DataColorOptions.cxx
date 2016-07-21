/*ckwg +29
 * Copyright 2016 by Kitware, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name Kitware, Inc. nor the names of any contributors may be
 *    used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHORS OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

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

  connect(d->UI.minimum, SIGNAL(valueChanged(double)),
          this, SLOT(updateMinimum()));
  connect(d->UI.maximum, SIGNAL(valueChanged(double)),
          this, SLOT(updateMaximum()));
  connect(d->UI.autoMinimum, SIGNAL(toggled(bool)),
          this, SLOT(updateMinimum()));
  connect(d->UI.autoMaximum, SIGNAL(toggled(bool)),
          this, SLOT(updateMaximum()));

  connect(d->UI.gradient, SIGNAL(currentIndexChanged(int)),
          this, SLOT(setGradient(int)));
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
  return d->scalarsToGradient.GetPointer();
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
