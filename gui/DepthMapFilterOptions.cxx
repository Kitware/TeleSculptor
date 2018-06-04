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

  double initialBCMin, initialBCMax, initialURMin, initialURMax;
};

//-----------------------------------------------------------------------------
DepthMapFilterOptions::DepthMapFilterOptions(
  const QString& settingsGroup, QWidget* parent, Qt::WindowFlags flags)
  : QWidget(parent, flags), d_ptr(new DepthMapFilterOptionsPrivate)
{
  QTE_D();

  d->UI.setupUi(this);

  connect(d->UI.bestCostMinimum, SIGNAL(valueChanged(double)),
          this, SLOT(updateBestCostMinimum()));
  connect(d->UI.bestCostMaximum, SIGNAL(valueChanged(double)),
          this, SLOT(updateBestCostMaximum()));
  connect(d->UI.uniquenessRatioMinimum, SIGNAL(valueChanged(double)),
          this, SLOT(updateUniquenessRatioMinimum()));
  connect(d->UI.uniquenessRatioMaximum, SIGNAL(valueChanged(double)),
          this, SLOT(updateUniquenessRatioMaximum()));

  connect(d->UI.buttonBox->button(QDialogButtonBox::Apply), SIGNAL(clicked()),
          this, SIGNAL(filtersChanged()));
  connect(d->UI.buttonBox->button(QDialogButtonBox::Reset), SIGNAL(clicked()),
          this, SLOT(resetFilters()));
}

//-----------------------------------------------------------------------------
DepthMapFilterOptions::~DepthMapFilterOptions()
{
}

//-----------------------------------------------------------------------------
double DepthMapFilterOptions::bestCostValueMinimum() const
{
  QTE_D();
  return d->UI.bestCostMinimum->value();
}

//-----------------------------------------------------------------------------
double DepthMapFilterOptions::bestCostValueMaximum() const
{
  QTE_D();
  return d->UI.bestCostMaximum->value();
}

//-----------------------------------------------------------------------------
double DepthMapFilterOptions::uniquenessRatioMinimum() const
{
  QTE_D();
  return d->UI.uniquenessRatioMinimum->value();
}

//-----------------------------------------------------------------------------
double DepthMapFilterOptions::uniquenessRatioMaximum() const
{
  QTE_D();
  return d->UI.uniquenessRatioMaximum->value();
}

//-----------------------------------------------------------------------------
void DepthMapFilterOptions::updateBestCostMinimum()
{
  QTE_D();

  auto const limit = d->UI.bestCostMinimum->value();
  if (d->UI.bestCostMaximum->value() < limit)
  {
    d->UI.bestCostMaximum->setValue(limit);
  }
}

//-----------------------------------------------------------------------------
void DepthMapFilterOptions::updateBestCostMaximum()
{
  QTE_D();

  auto const limit = d->UI.bestCostMaximum->value();
  if (d->UI.bestCostMinimum->value() > limit)
  {
    d->UI.bestCostMinimum->setValue(limit);
  }
}

//-----------------------------------------------------------------------------
void DepthMapFilterOptions::updateUniquenessRatioMinimum()
{
  QTE_D();

  auto const limit = d->UI.uniquenessRatioMinimum->value();
  if (d->UI.uniquenessRatioMaximum->value() < limit)
  {
    d->UI.uniquenessRatioMaximum->setValue(limit);
  }
}

//-----------------------------------------------------------------------------
void DepthMapFilterOptions::updateUniquenessRatioMaximum()
{
  QTE_D();

  auto const limit = d->UI.uniquenessRatioMaximum->value();
  if (d->UI.uniquenessRatioMinimum->value() > limit)
  {
    d->UI.uniquenessRatioMinimum->setValue(limit);
  }
}

//-----------------------------------------------------------------------------
void DepthMapFilterOptions::initializeFilters(double bcMin, double bcMax,
                                              double urMin, double urMax)
{
  QTE_D();

  d->initialBCMin = bcMin;
  d->initialBCMax = bcMax;
  d->initialURMin = urMin;
  d->initialURMax = urMax;

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

  qtScopedBlockSignals bbcl(d->UI.bestCostMinimum);
  qtScopedBlockSignals bbcu(d->UI.bestCostMaximum);
  qtScopedBlockSignals burl(d->UI.uniquenessRatioMinimum);
  qtScopedBlockSignals buru(d->UI.uniquenessRatioMaximum);

  d->UI.bestCostMinimum->setRange(d->initialBCMin, d->initialBCMax);
  d->UI.bestCostMaximum->setRange(d->initialBCMin, d->initialBCMax);
  d->UI.bestCostMinimum->setValue(d->initialBCMin);
  d->UI.bestCostMaximum->setValue(d->initialBCMax);

  d->UI.uniquenessRatioMinimum->setRange(d->initialURMin, d->initialURMax);
  d->UI.uniquenessRatioMaximum->setRange(d->initialURMin, d->initialURMax);
  d->UI.uniquenessRatioMinimum->setValue(d->initialURMin);
  d->UI.uniquenessRatioMaximum->setValue(d->initialURMax);
}
