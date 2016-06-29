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
 *  * Neither name of Kitware, Inc. nor the names of any contributors may be used
 *    to endorse or promote products derived from this software without specific
 *    prior written permission.
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

#include <qtUiState.h>
#include <qtUiStateItem.h>


//-----------------------------------------------------------------------------
class DepthMapFilterOptionsPrivate
{
public:
  DepthMapFilterOptionsPrivate() {}

  Ui::DepthMapFilterOptions UI;
  qtUiState uiState;

  double initialBCMin, initialBCMax, initialURMin, initialURMax;
};

QTE_IMPLEMENT_D_FUNC(DepthMapFilterOptions)

//-----------------------------------------------------------------------------
DepthMapFilterOptions::DepthMapFilterOptions(const QString &settingsGroup, QWidget* parent, Qt::WindowFlags flags)
  : QWidget(parent, flags), d_ptr(new DepthMapFilterOptionsPrivate)
{
  QTE_D();

  // Set up UI
  d->UI.setupUi(this);


  // Set up option persistence
  d->uiState.setCurrentGroup(settingsGroup);

  d->uiState.restore();

  // Connect signals/slots
  connect(d->UI.pushButtonApply, SIGNAL(pressed()),
          this, SLOT(updateFilters()));

//  connect(d->UI.doubleSpinBoxBestCostMax, SIGNAL(valueChanged(double)),
//          this, SLOT(updateFilters()));

//  connect(d->UI.doubleSpinBoxUniquenessRatioMin, SIGNAL(valueChanged(double)),
//          this, SLOT(updateFilters()));

//  connect(d->UI.doubleSpinBoxUniquenessRatioMax, SIGNAL(valueChanged(double)),
//          this, SLOT(updateFilters()));

  connect(d->UI.pushButtonReset, SIGNAL(pressed()),
          this, SLOT(resetFilters()));


}

//-----------------------------------------------------------------------------
DepthMapFilterOptions::~DepthMapFilterOptions()
{
  QTE_D();
  d->uiState.save();
}
//-----------------------------------------------------------------------------
double DepthMapFilterOptions::getBestCostValueMin()
{
  QTE_D();

 return d->UI.doubleSpinBoxBestCostMin->value();
}
//-----------------------------------------------------------------------------
double DepthMapFilterOptions::getBestCostValueMax()
{
  QTE_D();

 return d->UI.doubleSpinBoxBestCostMax->value();
}
//-----------------------------------------------------------------------------
double DepthMapFilterOptions::getUniquenessRatioMin()
{
  QTE_D();

 return d->UI.doubleSpinBoxUniquenessRatioMin->value();
}
//-----------------------------------------------------------------------------
double DepthMapFilterOptions::getUniquenessRatioMax()
{
  QTE_D();

  return d->UI.doubleSpinBoxUniquenessRatioMax->value();
}

void DepthMapFilterOptions::updateFilters()
{
  QTE_D();

  if(d->UI.doubleSpinBoxUniquenessRatioMax->value() < d->UI.doubleSpinBoxUniquenessRatioMin->value())
  {
    d->UI.doubleSpinBoxUniquenessRatioMax->setValue(d->UI.doubleSpinBoxUniquenessRatioMin->value());
  }

  if(d->UI.doubleSpinBoxBestCostMax->value() < d->UI.doubleSpinBoxBestCostMin->value())
  {
    d->UI.doubleSpinBoxBestCostMax->setValue(d->UI.doubleSpinBoxBestCostMin->value());
  }

  d->UI.doubleSpinBoxUniquenessRatioMax->setMinimum(d->UI.doubleSpinBoxUniquenessRatioMin->value());
  d->UI.doubleSpinBoxBestCostMax->setMinimum(d->UI.doubleSpinBoxBestCostMin->value());

  emit filtersChanged();

}

void DepthMapFilterOptions::initializeFilters(double bcMin, double bcMax, double urMin, double urMax)
{
  QTE_D();

  d->initialBCMin = bcMin;
  d->initialBCMax = bcMax;
  d->initialURMin = urMin;
  d->initialURMax = urMax;

  resetFilters();
}

void DepthMapFilterOptions::resetFilters()
{
  QTE_D();

  d->UI.doubleSpinBoxBestCostMin->setValue(d->initialBCMin);
  d->UI.doubleSpinBoxBestCostMax->setValue(d->initialBCMax);

  d->UI.doubleSpinBoxBestCostMin->setMinimum(d->initialBCMin);
  d->UI.doubleSpinBoxBestCostMax->setMinimum(d->initialBCMin);

  d->UI.doubleSpinBoxBestCostMin->setMaximum(d->initialBCMax);
  d->UI.doubleSpinBoxBestCostMax->setMaximum(d->initialBCMax);

  d->UI.doubleSpinBoxUniquenessRatioMin->setValue(d->initialURMin);
  d->UI.doubleSpinBoxUniquenessRatioMax->setValue(d->initialURMax);

  d->UI.doubleSpinBoxUniquenessRatioMin->setMinimum(d->initialURMin);
  d->UI.doubleSpinBoxUniquenessRatioMax->setMinimum(d->initialURMin);

  d->UI.doubleSpinBoxUniquenessRatioMin->setMaximum(d->initialURMax);
  d->UI.doubleSpinBoxUniquenessRatioMax->setMaximum(d->initialURMax);
}

