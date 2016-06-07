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

  double bcMin = d->UI.doubleSpinBoxBestCostMin->value();
  double bcMax = d->UI.doubleSpinBoxBestCostMax->value();
  double urMin = d->UI.doubleSpinBoxUniquenessRatioMin->value();
  double urMax = d->UI.doubleSpinBoxUniquenessRatioMax->value();

  if(bcMax < bcMin)
  {
    d->UI.doubleSpinBoxBestCostMax->setValue(bcMin);
  }

  if(urMax < urMin)
  {
    d->UI.doubleSpinBoxUniquenessRatioMax->setValue(urMin);
  }

  d->UI.doubleSpinBoxBestCostMax->setMinimum(bcMin);
  d->UI.doubleSpinBoxUniquenessRatioMax->setMinimum(urMin);

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
  QDoubleSpinBox* bcMinSpinBox = d->UI.doubleSpinBoxBestCostMin;
  QDoubleSpinBox* bcMaxSpinBox = d->UI.doubleSpinBoxBestCostMax;
  QDoubleSpinBox* urMinSpinBox = d->UI.doubleSpinBoxUniquenessRatioMin;
  QDoubleSpinBox* urMaxSpinBox = d->UI.doubleSpinBoxUniquenessRatioMax;

  bcMinSpinBox->setValue(d->initialBCMin);
  bcMaxSpinBox->setValue(d->initialBCMax);

  bcMinSpinBox->setMinimum(d->initialBCMin);
  bcMaxSpinBox->setMinimum(d->initialBCMin);

  bcMinSpinBox->setMaximum(d->initialBCMax);
  bcMaxSpinBox->setMaximum(d->initialBCMax);

  urMinSpinBox->setValue(d->initialURMin);
  urMaxSpinBox->setValue(d->initialURMax);

  urMinSpinBox->setMinimum(d->initialURMin);
  urMaxSpinBox->setMinimum(d->initialURMin);

  urMinSpinBox->setMaximum(d->initialURMax);
  urMaxSpinBox->setMaximum(d->initialURMax);
}

