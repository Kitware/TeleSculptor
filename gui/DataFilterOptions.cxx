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

  connect(d->UI.minimum, SIGNAL(valueChanged(double)),
          this, SLOT(updateMinimum()));
  connect(d->UI.maximum, SIGNAL(valueChanged(double)),
          this, SLOT(updateMaximum()));
  connect(d->UI.useMinimum, SIGNAL(toggled(bool)),
          this, SIGNAL(modified()));
  connect(d->UI.useMaximum, SIGNAL(toggled(bool)),
          this, SIGNAL(modified()));

  connect(d->UI.reset, SIGNAL(clicked()), this, SLOT(resetRange()));
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
