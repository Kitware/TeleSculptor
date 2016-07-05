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

#include "DepthMapOptions.h"
#include "ui_DepthMapOptions.h"

#include <qtUiState.h>
#include <qtUiStateItem.h>
#include <QWidgetAction>
#include <QMenu>

#include <vtkActor.h>

#include "DepthMapFilterOptions.h"

///////////////////////////////////////////////////////////////////////////////

//BEGIN DepthMapOptionsPrivate declaration

//-----------------------------------------------------------------------------
class DepthMapOptionsPrivate
{
public:
  Ui::DepthMapOptions UI;
  qtUiState uiState;

  DepthMapFilterOptions* filterOptions;

  void setPopup(QToolButton* button, QWidget* widget);

};

//END DepthMapOptionsPrivate definition

///////////////////////////////////////////////////////////////////////////////

//BEGIN DepthMapOptionsPrivate implementation

//-----------------------------------------------------------------------------
void DepthMapOptionsPrivate::setPopup(QToolButton* button, QWidget* widget)
{
  auto const proxy = new QWidgetAction(button);
  proxy->setDefaultWidget(widget);

  auto const menu = new QMenu(button);
  menu->addAction(proxy);

  button->setMenu(menu);
}

//END DepthMapOptionsPrivate implementation

///////////////////////////////////////////////////////////////////////////////

//BEGIN DepthMapOptions

QTE_IMPLEMENT_D_FUNC(DepthMapOptions)

//-----------------------------------------------------------------------------
DepthMapOptions::DepthMapOptions(QString const& settingsGroup,
                                 QWidget* parent, Qt::WindowFlags flags) :
  QWidget(parent, flags), d_ptr(new DepthMapOptionsPrivate)
{
  QTE_D();

  // Set up UI
  d->UI.setupUi(this);

  // Set up option persistence
  d->uiState.setCurrentGroup(settingsGroup);

  d->uiState.restore();

  d->filterOptions = new DepthMapFilterOptions(settingsGroup, this);
  d->setPopup(d->UI.toolButtonFilters, d->filterOptions);
  d->UI.toolButtonFilters->setEnabled(false);

  // Connect signals/slots
  connect(d->UI.radioPoints, SIGNAL(toggled(bool)),
          this, SIGNAL(depthMapRepresentationChanged()));

  connect(d->UI.radioSurfaces, SIGNAL(toggled(bool)),
          this, SIGNAL(depthMapRepresentationChanged()));

  connect(d->UI.checkBoxFilters, SIGNAL(toggled(bool)),
          this, SLOT(showFiltersMenu(bool)));

  connect(d->filterOptions, SIGNAL(filtersChanged()),
          this, SIGNAL(depthMapThresholdsChanged()));
}

//-----------------------------------------------------------------------------
DepthMapOptions::~DepthMapOptions()
{
  QTE_D();
  d->uiState.save();
}

//-----------------------------------------------------------------------------
void DepthMapOptions::showFiltersMenu(bool state)
{
  QTE_D();

  d->UI.toolButtonFilters->setEnabled(state);
}

//-----------------------------------------------------------------------------
void DepthMapOptions::enable()
{
  QTE_D();

  d->UI.radioPoints->setEnabled(true);
  d->UI.radioPoints->setChecked(true);
}

//-----------------------------------------------------------------------------
bool DepthMapOptions::isFilterPersistChecked()
{
  QTE_D();

  return d->filterOptions->isFilterPersist();
}

//-----------------------------------------------------------------------------
bool DepthMapOptions::isFilterChecked()
{
  QTE_D();

  return d->UI.checkBoxFilters->isChecked();
}

//-----------------------------------------------------------------------------
double DepthMapOptions::getBestCostValueMin()
{
  QTE_D();

  return d->filterOptions->getBestCostValueMin();
}

//-----------------------------------------------------------------------------
double DepthMapOptions::getBestCostValueMax()
{
  QTE_D();

  return d->filterOptions->getBestCostValueMax();
}

//-----------------------------------------------------------------------------
double DepthMapOptions::getUniquenessRatioMin()
{
  QTE_D();

  return d->filterOptions->getUniquenessRatioMin();
}

//-----------------------------------------------------------------------------
double DepthMapOptions::getUniquenessRatioMax()
{
  QTE_D();

  return d->filterOptions->getUniquenessRatioMax();
}

//-----------------------------------------------------------------------------
void DepthMapOptions::initializeFilters(double bcMin, double bcMax,
                                        double urMin, double urMax)
{
  QTE_D();

  d->filterOptions->initializeFilters(bcMin,bcMax,urMin,urMax);
}

//-----------------------------------------------------------------------------
bool DepthMapOptions::isPointsChecked()
{
  QTE_D();

  return d->UI.radioPoints->isChecked();
}

//-----------------------------------------------------------------------------
bool DepthMapOptions::isSurfacesChecked()
{
  QTE_D();

  return d->UI.radioSurfaces->isChecked();
}

//END DepthMapOptions
