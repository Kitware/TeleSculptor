/*ckwg +29
 * Copyright 2016-2018 by Kitware, Inc.
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

#include "DepthMapFilterOptions.h"

#include "ui_DepthMapFilterOptions.h"

#include <qtScopedValueChange.h>

#include <QPushButton>

QTE_IMPLEMENT_D_FUNC(DepthMapFilterOptions)

//-----------------------------------------------------------------------------
class DepthMapFilterOptionsPrivate
{
public:
  Ui::DepthMapFilterOptions UI;

  double initialWeightMin, initialWeightMax, initialUncertMin, initialUncertMax;
};

//-----------------------------------------------------------------------------
DepthMapFilterOptions::DepthMapFilterOptions(
  const QString& settingsGroup, QWidget* parent, Qt::WindowFlags flags)
  : QWidget(parent, flags), d_ptr(new DepthMapFilterOptionsPrivate)
{
  QTE_D();

  d->UI.setupUi(this);

  connect(d->UI.weightMinimum,
          QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &DepthMapFilterOptions::updateWeightMinimum);
  connect(d->UI.weightMaximum,
          QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &DepthMapFilterOptions::updateWeightMaximum);
  connect(d->UI.uncertaintyMinimum,
          QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &DepthMapFilterOptions::updateUncertaintyMinimum);
  connect(d->UI.uncertaintyMaximum,
          QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &DepthMapFilterOptions::updateUncertaintyMaximum);

  connect(d->UI.buttonBox->button(QDialogButtonBox::Apply),
          &QAbstractButton::clicked,
          this, &DepthMapFilterOptions::filtersChanged);
  connect(d->UI.buttonBox->button(QDialogButtonBox::Reset),
          &QAbstractButton::clicked,
          this, &DepthMapFilterOptions::resetFilters);
}

//-----------------------------------------------------------------------------
DepthMapFilterOptions::~DepthMapFilterOptions()
{
}

//-----------------------------------------------------------------------------
double DepthMapFilterOptions::weightMinimum() const
{
  QTE_D();
  return d->UI.weightMinimum->value();
}

//-----------------------------------------------------------------------------
double DepthMapFilterOptions::weightMaximum() const
{
  QTE_D();
  return d->UI.weightMaximum->value();
}

//-----------------------------------------------------------------------------
double DepthMapFilterOptions::uncertaintyMinimum() const
{
  QTE_D();
  return d->UI.uncertaintyMinimum->value();
}

//-----------------------------------------------------------------------------
double DepthMapFilterOptions::uncertaintyMaximum() const
{
  QTE_D();
  return d->UI.uncertaintyMaximum->value();
}

//-----------------------------------------------------------------------------
void DepthMapFilterOptions::updateWeightMinimum()
{
  QTE_D();

  auto const limit = d->UI.weightMinimum->value();
  if (d->UI.weightMaximum->value() < limit)
  {
    d->UI.weightMaximum->setValue(limit);
  }
}

//-----------------------------------------------------------------------------
void DepthMapFilterOptions::updateWeightMaximum()
{
  QTE_D();

  auto const limit = d->UI.weightMaximum->value();
  if (d->UI.weightMinimum->value() > limit)
  {
    d->UI.weightMinimum->setValue(limit);
  }
}

//-----------------------------------------------------------------------------
void DepthMapFilterOptions::updateUncertaintyMinimum()
{
  QTE_D();

  auto const limit = d->UI.uncertaintyMinimum->value();
  if (d->UI.uncertaintyMaximum->value() < limit)
  {
    d->UI.uncertaintyMaximum->setValue(limit);
  }
}

//-----------------------------------------------------------------------------
void DepthMapFilterOptions::updateUncertaintyMaximum()
{
  QTE_D();

  auto const limit = d->UI.uncertaintyMaximum->value();
  if (d->UI.uncertaintyMinimum->value() > limit)
  {
    d->UI.uncertaintyMinimum->setValue(limit);
  }
}

//-----------------------------------------------------------------------------
void DepthMapFilterOptions::initializeFilters(double wMin, double wMax,
                                              double uMin, double uMax)
{
  QTE_D();

  d->initialWeightMin = wMin;
  d->initialWeightMax = wMax;
  d->initialUncertMin = uMin;
  d->initialUncertMax = uMax;

  resetFilters();
}

//-----------------------------------------------------------------------------
bool DepthMapFilterOptions::isFilterPersistent() const
{
  QTE_D();
  return d->UI.persist->isChecked();
}

//-----------------------------------------------------------------------------
void DepthMapFilterOptions::resetFilters()
{
  QTE_D();

  qtScopedBlockSignals bbcl(d->UI.weightMinimum);
  qtScopedBlockSignals bbcu(d->UI.weightMaximum);
  qtScopedBlockSignals burl(d->UI.uncertaintyMinimum);
  qtScopedBlockSignals buru(d->UI.uncertaintyMaximum);

  d->UI.weightMinimum->setRange(d->initialWeightMin, d->initialWeightMax);
  d->UI.weightMaximum->setRange(d->initialWeightMin, d->initialWeightMax);
  d->UI.weightMinimum->setValue(d->initialWeightMin);
  d->UI.weightMaximum->setValue(d->initialWeightMax);

  d->UI.uncertaintyMinimum->setRange(d->initialUncertMin, d->initialUncertMax);
  d->UI.uncertaintyMaximum->setRange(d->initialUncertMin, d->initialUncertMax);
  d->UI.uncertaintyMinimum->setValue(d->initialUncertMin);
  d->UI.uncertaintyMaximum->setValue(d->initialUncertMax);
}
